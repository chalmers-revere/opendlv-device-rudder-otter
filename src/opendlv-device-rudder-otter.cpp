/*
 * Copyright (C) 2019 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <fstream>
#include <vector>
#include <thread>
#include <chrono>
#include <mutex>

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include "ue9.hpp"
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") 
      || 0 == commandlineArguments.count("ip")) {
    std::cerr << argv[0] 
      << " interfaces to the Otter rudder controller using TCP." << std::endl;
    std::cerr << "Usage:   " << argv[0] 
      << " --ip=<IP address for the LabJack UE9> "
      << "--porta=<The port A for the LabJack UE9, default 52360> "
      << "--portb=<The port B for the LabJack UE9, default 52361> "
      << "--cid=<OpenDLV session> [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --ip=192.168.0.100 --cid=111" 
      << std::endl;
    retCode = 1;
  } else {
    bool const VERBOSE{commandlineArguments.count("verbose") != 0};
    uint16_t const CID = std::stoi(commandlineArguments["cid"]);
    
    uint32_t const PORTA{(commandlineArguments["porta"].size() != 0) 
      ? static_cast<uint32_t>(std::stoi(commandlineArguments["porta"])) 
        : 52360};
    uint32_t const PORTB{(commandlineArguments["portb"].size() != 0) 
      ? static_cast<uint32_t>(std::stoi(commandlineArguments["portb"])) 
        : 52361};

    uint16_t FREQ = 10;
    std::string const IP = commandlineArguments["ip"];
    Ue9 ue9(IP, PORTA, PORTB);

    double angleTarget = 0.0;
    std::mutex groundSteeringAngleMutex;

    auto onGroundSteeringRequest{[&angleTarget, &groundSteeringAngleMutex](
        cluon::data::Envelope &&envelope)
      {
        std::lock_guard<std::mutex> lock(groundSteeringAngleMutex);
        auto groundSteeringRequest = 
          cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(
              std::move(envelope));
        angleTarget = groundSteeringRequest.groundSteering();
      }};

    auto atFrequency{[&ue9, &angleTarget, &groundSteeringAngleMutex, &VERBOSE](
        ) -> bool
      {
        std::lock_guard<std::mutex> lock(groundSteeringAngleMutex);
        double const directionLeft = 0.0;
        double const directionRight = 5.0;

        double const speedVoltageMax = 2.5;

        double const analogMin = 0.251941;
        double const analogMax = 0.348241;
        double const analogRange = analogMax - analogMin;
        double const steeringRangeHalf = 23.5;

        double const pidK = 0.25;
        double const errorThreshold = 1.0;

        double analogCurrent;
        ue9.ReadAin(0, &analogCurrent, 103, 12, 0, 0);
        double angle = steeringRangeHalf 
          * (2.0 * (analogCurrent - analogMin) / analogRange - 1.0);

        double error = angleTarget - angle;

        double direction;
        if (error > 0) {
          direction = directionLeft;
        } else {
          direction = directionRight;
        }

        double speedVoltage = 0.0;
        double absError = std::abs(error);
        if (absError > errorThreshold) {
          double controlSignal = pidK * absError;
          speedVoltage = (controlSignal < speedVoltageMax) 
            ? controlSignal : speedVoltageMax;
        }

        ue9.WriteDac(1, direction);

        if (speedVoltage < 5.0) {
          ue9.WriteDac(0, speedVoltage);
        }

        if (VERBOSE) {
          std::cout << "angle error: " << error << " (cur: " << angle 
            << ", tar: " << angleTarget << ") " << std::endl;
        }

        return true;
      }};

    cluon::OD4Session od4{CID};
    od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);
    od4.timeTrigger(FREQ, atFrequency);
    
    ue9.WriteDac(0, 0.0);
  }
  return retCode;
}

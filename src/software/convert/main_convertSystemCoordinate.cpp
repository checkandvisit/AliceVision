// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/geometry/SystemCoordinate.hpp>

#include <boost/progress.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace av = aliceVision;
namespace po = boost::program_options;
namespace fs = boost::filesystem;

/**
 * @brief Transform pose from Arcore Sfm to Alice
 *
 * @param argc
 * @param argv
 * @return int
 */
int aliceVision_main(int argc, char** argv)
{

    // Input Path
    std::string inputSfmPath;
    std::string outputSfmPath;

    std::string verboseLevel = av::system::EVerboseLevel_enumToString(av::system::Logger::getDefaultVerboseLevel());

    std::string inputSystemCoordinate = "alice";
    std::string outputSystemCoordinate = "alice";

    po::options_description allParams("AliceVision convertSfMFormat");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()("input,i", po::value<std::string>(&inputSfmPath)->required(),
                                 "Input Sfm ArCore file path.")(
        "output,o", po::value<std::string>(&outputSfmPath)->required(), "Path to save file in.");

    po::options_description logParams("Log parameters");
    logParams.add_options()("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
                            "verbosity level (fatal,  error, warning, info, debug, trace).");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()("inputSystem",
                                 po::value<std::string>(&inputSystemCoordinate)->default_value(inputSystemCoordinate),
                                 "Input System Coordinate (alice, pytorch, arcore)")(
        "outputSystem", po::value<std::string>(&outputSystemCoordinate)->default_value(outputSystemCoordinate),
        "Output System Coordinate (alice, pytorch, arcore)");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, allParams), vm);

        if(vm.count("help") || (argc == 1))
        {
            ALICEVISION_COUT(allParams);
            return EXIT_SUCCESS;
        }
        po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    av::system::Logger::get()->setLogLevel(verboseLevel);

    av::sfmData::SfMData sfmData;
    ALICEVISION_LOG_TRACE("Load SfMData file");
    if(!av::sfmDataIO::Load(sfmData, inputSfmPath, av::sfmDataIO::ESfMData(av::sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << inputSfmPath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    typedef std::pair<const av::IndexT, av::sfmData::CameraPose> IdCamPair;
    boost::progress_display progressBar(sfmData.getPoses().size(), std::cout, "Transform Local Poses\n");

    for(auto it = sfmData.getPoses().begin(); it != sfmData.getPoses().end(); it++)
    {
        IdCamPair& curPair = *it;

        av::geometry::Pose3 inPose = curPair.second.getTransform();
        av::geometry::Pose3 outPose =
            av::geometry::SystemCoordinate::transform(inPose, inputSystemCoordinate, outputSystemCoordinate);

        curPair.second.setTransform(outPose);
        ++progressBar;
    }

    if(av::sfmDataIO::Save(sfmData, outputSfmPath, av::sfmDataIO::ESfMData(av::sfmDataIO::ALL)))
        return EXIT_SUCCESS;

    ALICEVISION_LOG_ERROR("Can't save the output SfMData.");
    return EXIT_FAILURE;
}
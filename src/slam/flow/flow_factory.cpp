//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <flow/flow.h>
// #include <gperftools/profiler.h>

#include <mico/slam/flow/processors/BlockDatabaseMarkI.h>
#include <mico/slam/flow/processors/BlockEntityDatabase.h>
#include <mico/slam/flow/processors/BlockLoopClosure.h>
#include <mico/slam/flow/processors/BlockOdometryPhotogrammetry.h>
#include <mico/slam/flow/processors/BlockOdometryRGBD.h>
#include <mico/slam/flow/processors/BlockOptimizerCF.h>
#include <mico/slam/flow/savers/SaverEntity.h>
#include <mico/slam/flow/savers/SaverImage.h>
#include <mico/slam/flow/savers/SaverTrajectory.h>
#include <mico/slam/flow/misc/BlockDataframeInspector.h>
#include <mico/slam/flow/BlockQueuer.h>
#include <mico/slam/flow/CastBlocks.h>

using namespace flow;
using namespace mico;


extern "C" flow::PluginNodeCreator* factory(){
    flow::PluginNodeCreator *creator = new flow::PluginNodeCreator;

    // Casters
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockDataframeToPose>>(); },                     "Cast");
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockDataframeToCloud>>(); },                    "Cast");
    
    // SLAM
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockOdometryRGBD>>(); },                        "SLAM");
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockOdometryPhotogrammetry>>(); },              "SLAM");
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockDatabaseMarkI>>(); },                       "SLAM");
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockLoopClosure>>(); },                         "SLAM");
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockOptimizerCF>>(); },                         "SLAM");
    
    // Savers
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<SaverImage>>(); },                               "Savers");
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<SaverTrajectory>>(); },                          "Savers");
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<SaverEntity>>(); },                              "Savers");
    
    // Queuers
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockQueuer<QueuerTraitClusterframes>>>(); },    "Queuer");
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockQueuer<QueuerTraitColor>>>        (); },    "Queuer");

    // Misc
    creator->registerNodeCreator([](){ return std::make_unique<FlowVisualBlock<BlockDataframeInspector>>(); },                  "Misc");

    return creator;
}

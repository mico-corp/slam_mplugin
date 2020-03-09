//---------------------------------------------------------------------------------------------------------------------
//  mico
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92)
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

#include <mico/slam/flow/processors/BlockEntityDatabase.h>
#include <flow/Policy.h>
#include <flow/Outpipe.h>
#include <flow/DataFlow.h>

#include <sstream>

namespace mico{

    BlockEntityDatabase::BlockEntityDatabase(){ 
        createPipe("Entities", "v_entity");
        
        createPolicy({{"Entities", "v_entity"}});

        registerCallback({"Entities"}, 
                                [&](flow::DataFlow _data){
                                    if(idle_){
                                        idle_ = false;
                                        #ifdef HAS_DARKNET
                                        auto entities = _data.get<std::vector<std::shared_ptr<mico::Entity<pcl::PointXYZRGBNormal>>>>("v_entity"); 
                                        
                                        if(!entities_.empty()){
                                            for(auto queryE: entities){
                                                // check overlap
                                                auto eDfs = queryE->dfs();
                                                auto queryBoundingCube = queryE->boundingCube(eDfs[0]);
                                                for(auto trainE: entities_){
                                                    auto trainBoundingCube = trainE.second->boundingCube(eDfs[0]);
                                                    double minXOverlap = std::max(trainBoundingCube[0], queryBoundingCube[0]);
                                                    double minYOverlap = std::max(trainBoundingCube[1], queryBoundingCube[1]);
                                                    double minZOverlap = std::max(trainBoundingCube[2], queryBoundingCube[2]);
                                                    double maxXOverlap = std::min(trainBoundingCube[3], queryBoundingCube[3]);
                                                    double maxYOverlap = std::min(trainBoundingCube[4], queryBoundingCube[4]);
                                                    double maxZOverlap = std::min(trainBoundingCube[5], queryBoundingCube[5]);  
                                                }
                                            }
                                        }else{
                                            for(auto e: entities){
                                                entities_[e->id()] = e;
                                                // check overlapping here maybe
                                            }
                                        }

                                        getPipe("v_entity")->flush(entities);
                                        idle_ = true;
                                        #endif
                                    }
                                }
        );
    }

    // BlockEntityDatabase::~BlockEntityDatabase(){
    // } 

    bool BlockEntityDatabase::configure(std::unordered_map<std::string, std::string> _params){
        cjson::Json jParams;
        for(auto &param: _params){
            if(param.second == "")
                return false;
            if(param.first == "score"){
                std::istringstream istr(_params["score"]);
                float score;
                istr >> score;
                jParams["score"] = score;
            }
        }
    }
    
    std::vector<std::string> BlockEntityDatabase::parameters(){
        return {"score"};
    }
}

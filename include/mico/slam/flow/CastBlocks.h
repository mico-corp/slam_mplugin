
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

#ifndef MICO_KIDS_BLOCKS_CASTBLOCKS_H_
#define MICO_KIDS_BLOCKS_CASTBLOCKS_H_

#include <flow/Policy.h>
#include <flow/Outpipe.h>
#include <flow/Block.h>

#include <mico/slam/Dataframe.h>

#include <iostream>

namespace mico{
    //-----------------------------------------------------------------------------------------------------------------
    // DATAFRAME CASTERS
    class BlockDataframeToSomething: public flow::Block{
    public:
        BlockDataframeToSomething();

        // ~BlockDataframeToSomething(){};
    protected:
        bool idle_ = true;
        virtual std::any dataToget(mico::Dataframe<pcl::PointXYZRGBNormal>::Ptr &_df) = 0;
        virtual std::string tagToGet() = 0;

    };

    //-----------------------------------------------------------------------------------------------------------------
    class BlockDataframeToPose: public BlockDataframeToSomething{
    public:
        /// Get name of block
        virtual std::string name() const override { return "Dataframe -> Pose"; }
        BlockDataframeToPose() { createPipe("Pose","mat44"); }
        // ~BlockDataframeToPose(){};

    protected:
        virtual std::any dataToget(mico::Dataframe<pcl::PointXYZRGBNormal>::Ptr &_df) override { return _df->pose(); }
        
        virtual std::string tagToGet() override { return "Pose"; }
    };


    //-----------------------------------------------------------------------------------------------------------------
    class BlockDataframeToCloud: public BlockDataframeToSomething{
    public:
        /// Get name of block
        virtual std::string name() const override { return "Dataframe -> Cloud"; }
        BlockDataframeToCloud(){ createPipe("Cloud","cloud"); }
        // ~BlockDataframeToCloud(){};

    protected:
        virtual std::any dataToget(mico::Dataframe<pcl::PointXYZRGBNormal>::Ptr &_df) override { return _df->cloud(); }
        
        virtual std::string tagToGet() override { return "Cloud"; }
    };
}

#endif
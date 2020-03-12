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


#include <mico/slam/flow/misc/BlockPython.h>

#include <flow/Policy.h>
#include <flow/Outpipe.h>
#include <flow/DataFlow.h>
#include <chrono>
#include <iostream>
#include <fstream>

#include <QtWidgets>
#include <QPushButton>

#ifdef slots
#undef slots
#endif

#include <Python.h>
#include <boost/python.hpp>
#include <boost/python/dict.hpp>
#include <numpy/arrayobject.h>

#include <mico/slam/flow/misc/python/ConversionUtils.h>

#include <opencv2/opencv.hpp>


namespace mico{
    BlockPython::BlockPython(){
        pybind11::initialize_interpreter();
        gilReleaser_ = new pybind11::gil_scoped_release;

        // Interface constructor
        interfaceSelector_ = new InterfaceSelectorWidget("Python interface Selector");
        interfaceSelector_->callThisIfSmthChangeInside([this](){this->prepareInterfaces();});


        // Base layout
        blockInterpreter_ = new QGroupBox("");
        blockInterpreterLayout_ = new QVBoxLayout();
        blockInterpreter_->setLayout(blockInterpreterLayout_);
        
        pythonEditor_ = new QTextEdit;
        highlighter_ = new PythonSyntaxHighlighter(pythonEditor_->document());
        blockInterpreterLayout_->addWidget(pythonEditor_);
        runButton_ = new QPushButton("play");
        blockInterpreterLayout_->addWidget(runButton_);
        
        QWidget::connect(runButton_, &QPushButton::clicked, [this]() {
                flow::DataFlow data({}, [](flow::DataFlow _data){});
                this->runPythonCode(data, false);
            });

    }
    BlockPython::~BlockPython(){
        delete gilReleaser_;
        pybind11::finalize_interpreter();
    }


    QBoxLayout * BlockPython::creationWidget(){
        QBoxLayout *layout = new QVBoxLayout();
        layout->addWidget(interfaceSelector_);
        return layout;
    }

    void replaceAll(std::string& str, const std::string& from, const std::string& to) {
        if(from.empty())
            return;
        size_t start_pos = 0;
        while((start_pos = str.find(from, start_pos)) != std::string::npos) {
            str.replace(start_pos, from.length(), to);
            start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
        }
    }


    void BlockPython::prepareInterfaces(){
        // Clean previous policies and pipes if exist
        removePolicy();
        removePipes();


        outputInfo_ = interfaceSelector_->getInterfaces(InterfaceSelectorWidget::INTERFACE_TYPE::OUTPUT);
        for(auto &output: outputInfo_){
            createPipe(output.first, output.second);
        }

        inputInfo_ = interfaceSelector_->getInterfaces(InterfaceSelectorWidget::INTERFACE_TYPE::INPUT);
        if(inputInfo_.size() > 0){
            createPolicy(inputInfo_);

            std::vector<std::string> inTags;
            for(auto input: inputInfo_){
                inTags.push_back(input.first);
            }

            registerCallback(inTags, [&](flow::DataFlow _data){
                this->runPythonCode(_data, true);
            });
        }

    }

    void BlockPython::runPythonCode(flow::DataFlow _data, bool _useData){
        if(!idle_)
            return;
        
        idle_ = false;
        pybind11::gil_scoped_acquire gil;

        std::string pythonCode = pythonEditor_->toPlainText().toStdString();
        // pybind11::scoped_interpreter guard{}; // Not most efficient but safe

        try {
            auto locals = pybind11::dict();
            if(_useData) { // Encode inputs
                for(auto input:inputInfo_){
                    encodeInput(locals, _data, input.first, input.second);
                }
            }

            pybind11::exec(pythonCode, pybind11::globals(), locals);
    
            for(auto output:outputInfo_){
                flushPipe(locals, output.first, output.second);
            }        

        }catch(pybind11::error_already_set &_e){
            //std::cout << "Catched pybinds exception: " << _e.what() << "\n";
            _e.restore();
        }catch(const std::exception& _e){
            //std::cout << "Catched std exception: " << _e.what() << "\n";
        }

        idle_ = true;
    }


    void BlockPython::encodeInput(pybind11::dict _locals, flow::DataFlow _data, std::string _tag, std::string _typeTag){

        if(_typeTag == "int"){
            _locals[_tag.c_str()] = pybind11::int_(_data.get<int>(_tag));
        }else if(_typeTag == "float"){
            _locals[_tag.c_str()] = pybind11::float_(_data.get<float>(_tag));
        }else if(_typeTag == "vec3"){
            _locals[_tag.c_str()] = _data.get<Eigen::Vector3f>(_tag);
        }else if(_typeTag == "vec4"){
            _locals[_tag.c_str()] = _data.get<Eigen::Vector4f>(_tag);
        }else if(_typeTag == "mat44"){
            _locals[_tag.c_str()] = _data.get<Eigen::Matrix4f>(_tag);
        }else if(_typeTag == "image"){
            auto image = _data.get<cv::Mat>(_tag);
            if(image.channels() == 1){
                _locals[_tag.c_str()] = cv_mat_uint8_1c_to_numpy(image);
            }else if(image.channels() == 3){
                _locals[_tag.c_str()] = cv_mat_uint8_3c_to_numpy(image);
            }else{
                std::cout << "Unsupported image type conversion in Python block" << std::endl;
            }
        }else{
            std::cout << "Type " << _typeTag << " of label "<< _tag << " is not supported yet in python block." << ".It will be initialized as none. Please contact the administrators" << std::endl;
            return;
        }

    }

    void BlockPython::flushPipe(pybind11::dict _locals , std::string _tag, std::string _typeTag){
       
        if(_locals.contains(_tag.c_str())){
            if(_typeTag == "int"){
                getPipe(_tag)->flush(_locals[_tag.c_str()].cast<int>());
            }else if(_typeTag == "float"){
                getPipe(_tag)->flush(_locals[_tag.c_str()].cast<float>());
            }else if(_typeTag == "vec3"){
                getPipe(_tag)->flush(_locals[_tag.c_str()].cast<Eigen::Vector3f>());
            }else if(_typeTag == "vec4"){
                getPipe(_tag)->flush(_locals[_tag.c_str()].cast<Eigen::Vector4f>());
            }else if(_typeTag == "mat44"){
                getPipe(_tag)->flush(_locals[_tag.c_str()].cast<Eigen::Matrix4f>());
            }else if(_typeTag == "image"){
                auto pyArray = pybind11::array_t<unsigned char>(_locals[_tag.c_str()]);
                if(pyArray.ndim() == 2){
                    getPipe(_tag)->flush(numpy_uint8_1c_to_cv_mat(pyArray));
                }else if(pyArray.ndim() == 3){
                    getPipe(_tag)->flush(numpy_uint8_3c_to_cv_mat(pyArray));
                }else{
                    std::cout << "Unsupported image type conversion in Python block" << std::endl;
                }
            }else{
                std::cout << "Type " << _typeTag << " of label "<< _tag << " is not supported yet in python block." << ".It will be initialized as none. Please contact the administrators" << std::endl;
            }
        }

        


    }
}

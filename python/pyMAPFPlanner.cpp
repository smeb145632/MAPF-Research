#include"pyMAPFPlanner.hpp"
#include <thread>
#include <future>
#include <unistd.h>

pyMAPFPlanner::pyMAPFPlanner():MAPFPlanner(){
    pybind11::gil_scoped_acquire acquire;

    py_env = new pyEnvironment(env);

    auto sys=pybind11::module_::import("sys");
    std::ifstream configFile("config.json");
    
    std::cout<<"setting to default python path: ./python, ../python, ./build "<<std::endl;
    sys.attr("path").attr("append")("./python");
    sys.attr("path").attr("append")("../python");
    sys.attr("path").attr("append")("./build");
    
    if(configFile){
        nlohmann::json configData;
        try{
            configFile>>configData;
            if(configData.contains("python_path")){
                std::string python_path=configData["python_path"];
                std::cout<<"addinng "<<python_path<<" to system path"<<std::endl;
                sys.attr("path").attr("append")(python_path);
            }
        }
        catch(const nlohmann::json::parse_error& e){
            std::cerr << "Error: Failed to parse config file. " << e.what() << std::endl;
        }
    }
    
    std::cout<<"trying to import pyMAPFPlanner module"<<std::endl;
    py_planner = new pybind11::object(
        pybind11::module_::import("pyMAPFPlanner").attr("pyMAPFPlanner")(py_env));

    std::cout<<"pyMAPF Planner Created! GIL="<<PyGILState_Check()<<std::endl;
    std::cout.flush();
    // GIL released here when acquire goes out of scope
    // py_planner is now a raw pointer - destruction is manual
}

pyMAPFPlanner::~pyMAPFPlanner(){
    // GIL acquired here for safe cleanup
    pybind11::gil_scoped_acquire acquire;
    if(py_planner != nullptr){
        delete py_planner;
        py_planner = nullptr;
    }
    delete py_env;
    py_env = nullptr;
}

void pyMAPFPlanner::initialize(int preprocess_time_limit){
    std::cout<<"pyMAPFPlanner initialize() called, GIL="<<PyGILState_Check()<<std::endl;
    std::cout.flush();
    
    // First release any existing GIL state, then acquire fresh
    pybind11::gil_scoped_acquire acquire;
    std::cout<<"pyMAPFPlanner initialize() GIL acquired"<<std::endl;
    std::cout.flush();
    
    py_planner->attr("initialize")(preprocess_time_limit);
    
    std::cout<<"pyMAPFPlanner initialize() done"<<std::endl;
    std::cout.flush();
}

void pyMAPFPlanner::plan(int time_limit,std::vector<Action> &plan){
    // Use PyGILState_Ensure which always works correctly
    PyGILState_STATE gstate = PyGILState_Ensure();
    std::cout<<"plan() GIL acquired, state="<<gstate<<std::endl;
    std::cout.flush();

    auto action_object=py_planner->attr("plan")(time_limit);
    try{
        plan=action_object.cast<std::vector<Action>>();
    }   
    catch(pybind11::cast_error e){
        plan.clear();
        std::vector<int> tmp_action=action_object.cast<std::vector<int>>();
        for(auto &ai:tmp_action){
            plan.push_back(static_cast<Action>(ai));
        }
    }
    
    PyGILState_Release(gstate);
    std::cout<<"plan() done"<<std::endl;
    std::cout.flush();
}

#ifndef PARAMETER_SERVER_HPP
#define PARAMETER_SERVER_HPP

#include <unordered_map>
#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>

namespace hdl_graph_slam {

struct ParameterServer {
public:
    template<typename T>
    T param(const std::string& name, const T& defalut_value) {
        auto found = params.find(name);
        if(found == params.end()) {
            params.insert(std::make_pair(name, defalut_value));
            found = params.find(name);

        }

        if(found->second.type() == typeid(T)) {
            return boost::any_cast<T>(found->second);
        } else if(found->second.type() == typeid(std::string)) {
            auto str = boost::any_cast<std::string>(found->second);
            T p = boost::lexical_cast<T>(str);
            found->second = p;
            return p;
        }

        std::cerr << "warning: param " << name << "'s type does not match!!" << std::endl;
        return defalut_value;
    }

private:
    std::unordered_map<std::string, boost::any> params;
};

}

#endif
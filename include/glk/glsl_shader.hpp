#ifndef GLK_GLSL_SHADER_HPP
#define GLK_GLSL_SHADER_HPP

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <unordered_map>

#include <GL/glew.h>
#include <Eigen/Dense>

namespace glk {

class GLSLShader {
public:
    GLSLShader()
    {}

    bool init(const std::string& shader_path) {
        GLuint vertex_shader = read_shader_from_file(shader_path + ".vert", GL_VERTEX_SHADER);
        GLuint fragment_shader = read_shader_from_file(shader_path + ".frag", GL_FRAGMENT_SHADER);

        shader_program = glCreateProgram();
        glAttachShader(shader_program, vertex_shader);
        glAttachShader(shader_program, fragment_shader);
        glLinkProgram(shader_program);

        glDeleteShader(vertex_shader);
        glDeleteShader(fragment_shader);

        GLint result = GL_FALSE;
        int info_log_length;

        glGetProgramiv(shader_program, GL_LINK_STATUS, &result);
        glGetProgramiv(shader_program, GL_INFO_LOG_LENGTH, &info_log_length);
        std::vector<char> error_message(info_log_length);
        glGetProgramInfoLog(shader_program, info_log_length, nullptr, error_message.data());

        if(result != GL_TRUE) {
            std::cerr << "error : failed to link program" << std::endl;
            std::cerr << std::string(error_message.begin(), error_message.end()) << std::endl;
            return false;
        }

        return true;
    }

    void use() const {
        glUseProgram(shader_program);
    }

    GLint attrib(const std::string& name) {
        auto found = attrib_cache.find(name);
        if(found != attrib_cache.end()) {
            return found->second;
        }

        GLint id = glGetAttribLocation(shader_program, name.c_str());
        if(id == -1) {
            std::cerr << "warning : attrib " << name << " not found" << std::endl;
        }

        attrib_cache[name] = id;
        return id;
    }

    GLint uniform(const std::string& name) {
        auto found = uniform_cache.find(name);
        if(found != uniform_cache.end()) {
            return found->second;
        }

        GLint id = glGetUniformLocation(shader_program, name.c_str());
        if(id == -1) {
            std::cerr << "warning : uniform " << name << " not found" << std::endl;
        }

        uniform_cache[name] = id;
        return id;
    }

    Eigen::Matrix4f get_uniform(const std::string& name) {
        Eigen::Matrix4f mat;
        glGetUniformfv(shader_program, uniform(name), mat.data());
        return mat;
    }

    void set_uniform(const std::string& name, int value) {
        glUniform1i(uniform(name), value);
    }

    void set_uniform(const std::string& name, float value) {
        glUniform1f(uniform(name), value);
    }

    void set_uniform(const std::string& name, const Eigen::Vector2f& vector) {
        glUniform2fv(uniform(name), 1, vector.data());
    }

    void set_uniform(const std::string& name, const Eigen::Vector4f& vector) {
        glUniform4fv(uniform(name), 1, vector.data());
    }

    void set_uniform(const std::string& name, const Eigen::Vector4i& vector) {
        glUniform4iv(uniform(name), 1, vector.data());
    }

    void set_uniform(const std::string& name, const Eigen::Matrix4f& matrix) {
        glUniformMatrix4fv(uniform(name), 1, GL_FALSE, matrix.data());
    }


private:
    GLuint read_shader_from_file(const std::string& filename, GLuint shader_type) {
        GLuint shader_id = glCreateShader(shader_type);
        std::ifstream ifs(filename);
        if(!ifs) {
            std::cerr << "error: failed to open " << filename << std::endl;
            return GL_FALSE;
        }

        std::stringstream sst;
        sst << ifs.rdbuf();
        std::string source = sst.str();

        GLint result = GL_FALSE;
        int info_log_length = 0;

        char const* source_ptr = source.c_str();
        glShaderSource(shader_id, 1, &source_ptr, nullptr);
        glCompileShader(shader_id);

        glGetShaderiv(shader_id, GL_COMPILE_STATUS, &result);
        glGetShaderiv(shader_id, GL_INFO_LOG_LENGTH, &info_log_length);

        std::vector<char> error_message(info_log_length);
        glGetShaderInfoLog(shader_id, info_log_length, nullptr, error_message.data());

        if(result != GL_TRUE) {
            std::cerr << "error : failed to compile shader" << std::endl;
            std::cerr << std::string(error_message.begin(), error_message.end()) << std::endl;
        }

        return shader_id;
    }


private:
    GLuint shader_program;
    std::unordered_map<std::string, GLint> attrib_cache;
    std::unordered_map<std::string, GLint> uniform_cache;
};

}

#endif
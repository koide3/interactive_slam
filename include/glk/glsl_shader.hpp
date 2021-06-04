#ifndef GLK_GLSL_SHADER_HPP
#define GLK_GLSL_SHADER_HPP

#include <memory>
#include <string>
#include <unordered_map>

#include <GL/gl3w.h>
#include <Eigen/Core>

namespace glk {

/**
 * @brief GLSL shader class
 *
 */
class GLSLShader {
public:
  GLSLShader() {}

  /**
   * @brief load GLSL shader from files
   *
   * @param shader_path  "shader_path".vert and "shader_path".frag will be loaded
   * @return             if the shader is successfully loaded
   */
  bool init(const std::string& shader_path);

  /**
   * @brief bind the shader
   */
  void use() const { glUseProgram(shader_program); }

  /** @brief find attribute variable location **/
  GLint attrib(const std::string& name);
  /** @brief find uniform variable location **/
  GLint uniform(const std::string& name);


  /*** getter and setter for uniforms ***/
  Eigen::Vector4f get_uniform4f(const std::string& name) {
    Eigen::Vector4f vec;
    glGetUniformfv(shader_program, uniform(name), vec.data());
    return vec;
  }

  Eigen::Matrix4f get_uniform_matrix4f(const std::string& name) {
    Eigen::Matrix4f mat;
    glGetUniformfv(shader_program, uniform(name), mat.data());
    return mat;
  }

  float get_uniform1f(const std::string& name){
    float res;
    glGetUniformfv(shader_program, uniform(name), &res);
    return res;
  }

  void set_uniform(const std::string& name, int value) { glUniform1i(uniform(name), value); }
  void set_uniform(const std::string& name, float value) { glUniform1f(uniform(name), value); }
  void set_uniform(const std::string& name, const Eigen::Vector2f& vector) { glUniform2fv(uniform(name), 1, vector.data()); }
  void set_uniform(const std::string& name, const Eigen::Vector3f& vector) { glUniform3fv(uniform(name), 1, vector.data()); }
  void set_uniform(const std::string& name, const Eigen::Vector4f& vector) { glUniform4fv(uniform(name), 1, vector.data()); }
  void set_uniform(const std::string& name, const Eigen::Vector4i& vector) { glUniform4iv(uniform(name), 1, vector.data()); }
  void set_uniform(const std::string& name, const Eigen::Matrix4f& matrix) { glUniformMatrix4fv(uniform(name), 1, GL_FALSE, matrix.data()); }

private:
  GLuint read_shader_from_file(const std::string& filename, GLuint shader_type);

private:
  GLuint shader_program;
  std::unordered_map<std::string, GLint> attrib_cache;
  std::unordered_map<std::string, GLint> uniform_cache;
};

}  // namespace glk

#endif
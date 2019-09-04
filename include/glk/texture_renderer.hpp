#ifndef GLK_TEXTURE_RENDERER_HPP
#define GLK_TEXTURE_RENDERER_HPP

#include <GL/glew.h>
#include <glk/glsl_shader.hpp>

namespace glk {

class TextureRenderer {
public:
    TextureRenderer() {
        std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices = {
            Eigen::Vector3f(-1.0f, -1.0f, 0.0f),
            Eigen::Vector3f( 1.0f, -1.0f, 0.0f),
            Eigen::Vector3f(-1.0f,  1.0f, 0.0f),
            Eigen::Vector3f( 1.0f,  1.0f, 0.0f)
        };

        glGenVertexArrays(1, &vao);
        glBindVertexArray(vao);

        glGenBuffers(1, &vbo);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GL_FLOAT) * 3 * vertices.size(), vertices.data(), GL_STATIC_DRAW);

        if(!shader.init("data/shader/texture")) {
            return;
        }

        shader.use();
        GLint position_loc = shader.attrib("position");

        glBindVertexArray(vao);
        glEnableVertexAttribArray(position_loc);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        // glDisableVertexAttribArray(0);
    }

    ~TextureRenderer() {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
    }

    void draw(GLuint texture) {
        shader.use();

        glBindTexture(GL_TEXTURE_2D, texture);

        glBindVertexArray(vao);
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }

private:
    GLuint vao;
    GLuint vbo;

    glk::GLSLShader shader;
};

}

#endif
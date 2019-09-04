#ifndef GLK_FRAME_BUFFER_HPP
#define GLK_FRAME_BUFFER_HPP

#include <GL/glew.h>
#include <Eigen/Dense>

#include <glk/texture.hpp>

namespace glk {



class FrameBuffer {
public:
    FrameBuffer(const Eigen::Vector2i& size, int num_color_buffers=1)
    : width(size[0]),
    height(size[1])
    {
        for(int i=0; i<num_color_buffers; i++) {
            if(i==0) {
                color_buffers.push_back(std::make_shared<Texture>(size, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE));
            } else {
                color_buffers.push_back(std::make_shared<Texture>(size, GL_RGBA32I, GL_RGBA_INTEGER, GL_INT));
            }
        }
        depth_buffer = std::make_shared<Texture>(size, GL_DEPTH_COMPONENT32F, GL_DEPTH_COMPONENT, GL_FLOAT);

        glGenFramebuffers(1, &frame_buffer);
        glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);

        GLenum color_attachments[] = { GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2 };
        for(int i=0; i<num_color_buffers; i++) {
            glFramebufferTexture2D(GL_FRAMEBUFFER, color_attachments[i], GL_TEXTURE_2D, color_buffers[i]->id(), 0);
        }
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_buffer->id(), 0);

        glDrawBuffers(num_color_buffers, color_attachments);

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    ~FrameBuffer() {
        glDeleteFramebuffers(1, &frame_buffer);
    }

    void bind() {
        glGetIntegerv(GL_VIEWPORT, viewport);
        glViewport(0, 0, width, height);
        glBindFramebuffer(GL_FRAMEBUFFER, frame_buffer);
    }

    void unbind() const {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    }

    const Texture& color() { return *color_buffers[0]; }
    const Texture& color(int i) { return *color_buffers[i]; }
    const Texture& depth() { return *depth_buffer; }

private:
    int width;
    int height;

    GLint viewport[4];

    std::vector<std::shared_ptr<Texture>> color_buffers;
    std::shared_ptr<Texture> depth_buffer;

    GLuint frame_buffer;
};

}

#endif
//----------------------------------------------------------------------------------------
/**
 * \file       Shader.h
 * \brief      Class that controls shader.
 *
 *  Compiles and stores shader object and resolves all uniforms bindings.
 *
 */
//----------------------------------------------------------------------------------------


#ifndef SHADER_H
#define SHADER_H
#include <string>
#include <filesystem>

class Shader {
public:
    /**
     * Creates classic shader object from shader pipeline with vertex and fragment shaders, store ID inside the object
     * @param vs vertex shader path
     * @param fs fragment shader path
     */
    Shader(const char *vs, const char *fs) {
        GLint linked;
        GLuint vertexShader = shaderCompiler(GL_VERTEX_SHADER, fileReader(vs).c_str());
        GLuint fragmentShader = shaderCompiler(GL_FRAGMENT_SHADER, fileReader(fs).c_str());
        ID = glCreateProgram();

        glAttachShader(ID, vertexShader);
        glAttachShader(ID, fragmentShader);
        glLinkProgram(ID);
        glGetProgramiv(ID, GL_LINK_STATUS, &linked);
        if (!linked) {
            spdlog::error("Shader program failed to link {}, {}", vs, fs);
            throw std::runtime_error("Shader linking error");
        }

        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
    }

    /**
     * Creates compute shader object, store ID inside the object
     * @param cs compute shader path
     */
    explicit Shader(const char *cs) {
        GLint linked;
        const GLuint computeShader = shaderCompiler(GL_COMPUTE_SHADER, fileReader(cs).c_str());
        ID = glCreateProgram();
        glAttachShader(ID, computeShader);
        glLinkProgram(ID);
        glGetProgramiv(ID, GL_LINK_STATUS, &linked);
        if (!linked) {
            spdlog::error("Compute shader program failed to link {}", cs);
            throw std::runtime_error("Shader linking error");
        }
        glDeleteShader(computeShader);
    }

    /**
     * Sets opengl state to use shader of current object
     */
    void use() const {
        glUseProgram(ID);
    }

    ~Shader(){
        glDeleteProgram(ID);
    }

    /**
     * Sets uniform for that shader object
     * @tparam T float/int/uint/vec3/mat4/ivec3/ivec2/bool
     * @param name name in the shader
     * @param value
     */
    template<typename T>
    void setUniform(const char *name, const T &value) const{
        GLint location = glGetUniformLocation(ID, name);

        if constexpr (std::is_same_v<T, float>) {
            glUniform1f(location, value);
        }
        else if constexpr (std::is_same_v<T, int>) {
            glUniform1i(location, value);
        }
        else if constexpr (std::is_same_v<T, uint>) {
            glUniform1ui(location, value);
        }
        else if constexpr (std::is_same_v<T, glm::vec3>) {
            glUniform3fv(location, 1, &value[0]); //uniform3fv takes first value of vec3
        }
        else if constexpr (std::is_same_v<T, glm::mat4>) {
            glUniformMatrix4fv(location, 1, GL_FALSE, &value[0][0]);
        }
        else if constexpr (std::is_same_v<T, glm::ivec3>) {
            glUniform3iv(location, 1, &value[0]);
        }
        else if constexpr (std::is_same_v<T, glm::ivec2>) {
            glUniform2iv(location, 1, &value[0]);
        }
        else if constexpr (std::is_same_v<T, bool>) {
            glUniform1i(location, value);
        }
        else {
            static_assert(!sizeof(T), "Unsupported uniform type");
        }
    }

private:
    /**
     * Compiles shader from shader source
     * @param type compute/vertex/fragment
     * @param shaderSource shader as string
     * @return returns compiled shader program
     */
    GLuint shaderCompiler(GLenum type, const char * shaderSource) {
        GLuint shader = glCreateShader(type);
        glShaderSource(shader, 1, &shaderSource, nullptr);
        glCompileShader(shader);

        GLint compiled;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);

        if (!compiled) {
            GLint len = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);

            std::string log(len, '\0');
            glGetShaderInfoLog(shader, len, nullptr, log.data());
            spdlog::error("Shader compilation failed {} {}", type, log);
            throw std::runtime_error("Shader compilation error");
        }
        return shader;
    }

    /**
     * Reads file from the path
     * @param filePath path to the file
     * @return string with the file contents
     */
    std::string fileReader(const char* filePath) {
        std::ifstream file(filePath, std::ios::in | std::ios::binary);

        // If shader not found, repair path with defined macro from CMake
        if (!file && std::string(filePath).find("..") != std::string::npos) {
#ifdef EXTERNAL_SHADER_PATH
        // Parse file name
        std::string originalPath = filePath;
        size_t pos = originalPath.find("shaders/");
        std::string relativeSubPath = (pos != std::string::npos) 
                                      ? originalPath.substr(pos + 8) // +8 skip "shaders/"
                                      : originalPath;

        std::string absolutePath = std::string(EXTERNAL_SHADER_PATH) + "/" + relativeSubPath;
        
        // Try once again
        file.open(absolutePath, std::ios::in | std::ios::binary);
#endif
    }

        if (!file) {
            spdlog::error("File cannot be opened: {}", filePath);
            throw std::runtime_error("File reading error at shader compilation");
        }

        std::ostringstream contents;
        contents << file.rdbuf();
        file.close();

        return contents.str();
    }

    GLuint ID;
};



#endif //SHADER_H

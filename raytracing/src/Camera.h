//----------------------------------------------------------------------------------------
/**
 * \file       Camera.h
 * \brief      Camera object
 *
 *  Camera object.
 *
 */
//----------------------------------------------------------------------------------------

#ifndef CAMERA_H
#define CAMERA_H

class Camera {
public:
    /**
     * Creates standard camera, optionally using a specific position and target
     * @param pos Initial camera position
     * @param tgt Initial camera target
     */
    Camera(glm::vec3 pos = glm::vec3(0.0f, 1.5f, -2.0f), glm::vec3 tgt = glm::vec3(0.0f, 0.0f, 0.0f)) {
        cameraPos = pos;
        target = tgt;
        glm::vec3 offset = cameraPos - target;
        radius = glm::length(offset);
        yaw = atan2(offset.x, offset.z);
        pitch = asinf(offset.y / radius);
        worldUp = glm::vec3(0.0f, 1.0f, 0.0f);
        camForward = glm::normalize(target - cameraPos);
        camRight = glm::normalize(glm::cross(camForward, worldUp));
        camUp = glm::cross(camRight, camForward);
        viewAngle = 45.0f;
    }

    /**
     * Returns view matrix of the camera while recounting place from moving around the simulation
     * @return mat4 view
     */
    [[nodiscard]] glm::mat4 getView() {
        pitch = glm::clamp(pitch, -glm::radians(89.0f), glm::radians(89.0f));
        glm::vec3 offset(radius * cosf(pitch) * sinf(yaw), radius * sinf(pitch), radius * cosf(pitch) * cosf(yaw));

        cameraPos = target + offset;
        camForward = glm::normalize(target - cameraPos);
        camRight = glm::normalize(glm::cross(camForward, worldUp));
        camUp = glm::cross(camRight, camForward);
        return glm::lookAt(cameraPos, target, worldUp);
    }

    /**
     * Returns project matrix
     * @return mat4 proj
     */
    [[nodiscard]] glm::mat4 getProj() const {
        return glm::perspective(glm::radians(viewAngle), aspect, nearP, farP);
    }

    /**
     * Return inversed view matrix
     * @return mat4 invView
     */
    [[nodiscard]] glm::mat4 getInvView() {
        return glm::inverse(getView());
    }

    /**
     * Return inversed proj matrix
     * @return mat4 invProj
     */
    [[nodiscard]] glm::mat4 getInvProj() const {
        return glm::inverse(getProj());
    }

    /**
     * Sets camera aspect according to framebuffer size
     * @param ww width of the framebuffer
     * @param wh height of the framebuffer
     */
    void setAspect(int ww, int wh) {
        aspect = static_cast<float>(ww) / static_cast<float>(wh);
    }

    /**
     * Sets camera position, used in movement
     * @param pos new camera pos in 3D space
     */
    void setPos(glm::vec3 pos) {
        cameraPos = pos;
        glm::vec3 offset = cameraPos - target;
        radius = glm::length(offset);
        yaw = atan2(offset.x, offset.z);
        pitch = asinf(offset.y / radius);
    }

    glm::vec3 cameraPos;
    glm::vec3 target;
    glm::vec3 worldUp;
    glm::vec3 camForward;
    glm::vec3 camRight;
    glm::vec3 camUp;
    float viewAngle;
    float nearP = 0.01f;
    float farP = 100.0f;
    float aspect = 16.0/9.0;
    float yaw = 0.0f;
    float pitch = 0.0f;
    float radius = 0.0f;
    bool rot = false;
    float sense = 0.005f;
    double lastX = 0.0f;
    double lastY = 0.0f;
};



#endif //CAMERA_H

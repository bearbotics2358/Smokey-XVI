#pragma once

#include <optional>
//#include <photonlib/PhotonCamera.h>
#include <string>
#include <vector>

#include "types.h"

struct Target {
        float angle;
        float distance;
};

enum class TargetType {
    Red,
    Blue
};

class TargetTracker {
    public:
        class Mode {
            public:
                // creates a mode with target mode and sets the index to pipelineIndex
                static Mode target(int pipelineIndex);

                // creates a mode with ball mode and sets the index to pipelineIndex
                static Mode ball(int pipelineIndex);

                int getPipelineIndex() const;

                bool isTarget() const;

                bool isBall() const;

            private:
                enum class InnerMode {
                    // track target to shoot into
                    Target,
                    // track balls
                    Ball
                };

                Mode(InnerMode mode, int pipelineIndex);

                InnerMode m_innerMode;
                int m_pipelineIndex;
        };

        // takes in the mdns name of the camera
        // this one sets team to read by defaualt
        TargetTracker(const std::string& cameraName, TargetTracker::Mode mode);

        // this one takes in a team
        TargetTracker(const std::string& cameraName, TargetTracker::Mode mode, TargetType team);

        void setTeam(TargetType team);
        void setMode(TargetTracker::Mode mode);

        void update();

        std::optional<Target> getTarget() const;
        // TODO: maybe get a way to return the enemy team balls
        const std::vector<Target>& getBalls() const;

    private:
        //photonlib::PhotonCamera m_camera;
        TargetTracker::Mode m_mode;
        TargetType m_team;

        std::optional<Target> m_target {};
        std::vector<Target> m_balls {};
};
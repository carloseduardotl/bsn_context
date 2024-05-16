#ifndef CONTEXTAPAPTATION_HPP
#define CONTEXTAPAPTATION_HPP

#include <ros/ros.h>
#include <ros/console.h>
#include <string>

#include "archlib/ROSComponent.hpp"
#include "libbsn/utils/utils.hpp"
#include "messages/TargetSystemData.h"
#include "services/PatientAdapt.h"

struct risk_values {
    float lowRisk[2];
    float midRisk0[2];
    float midRisk1[2];
    float highRisk0[2];
    float highRisk1[2];
};

class ContextAdaptation : public arch::ROSComponent {
    public:
        ContextAdaptation(int &argc, char **argv, std::string name);
        ~ContextAdaptation();

        void setUp();
        void tearDown();
        void body();

    private:

        void collect(const messages::TargetSystemData::ConstPtr& msg);
        ros::NodeHandle nh;

        void setUpContext();

        risk_values heart_rate_context[3];

        bool setRisks(std::string vitalSign,float* lowRisk, float* MidRisk0, float* MidRisk1, float* highRisk0, float* highRisk1);

        void monitor();
        void analyze();
        void plan();
        void execute();

};

#endif
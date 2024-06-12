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

struct TargetSystemData {
    double trm_risk;
    double ecg_risk;
    double oxi_risk;
    double abps_risk;
    double abpd_risk;
    double glc_risk;
    double trm_data;
    double ecg_data;
    double oxi_data;
    double abps_data;
    double abpd_data;
    double glc_data;
    double patient_status;
    bool pending_analysis;
};

class ContextAdaptation : public arch::ROSComponent {
    public:
        ContextAdaptation(int &argc, char **argv, std::string name);
        ~ContextAdaptation();

        void setUp();
        void tearDown();
        void body();

    private:

        void monitor(const messages::TargetSystemData::ConstPtr& msg);
        ros::NodeHandle nh;
        int currentContext;

        void setUpContext();
        void updateRiskValues(risk_values& context, const std::string& risk, const std::vector<std::string>& values);
        void printAllRiskValues();
        void printRiskValues(const risk_values& values, const std::string& vitalName);


        risk_values heartRateContext[3];
        risk_values oxigenationContext[3];
        TargetSystemData currentData;

        bool setRisks(std::string vitalSign,float* lowRisk, float* MidRisk0, float* MidRisk1, float* highRisk0, float* highRisk1);

        //void monitor();
        void analyze();
        void plan();
        void execute();

};

#endif
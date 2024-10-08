#ifndef CONTEXTAPAPTATION_HPP
#define CONTEXTAPAPTATION_HPP

#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <unordered_map>
#include <queue>
#include <vector>

#include "archlib/ROSComponent.hpp"
#include "libbsn/utils/utils.hpp"
#include "messages/TargetSystemData.h"
#include "services/PatientAdapt.h"

struct RiskValues {
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
        int riskThreshold;
        int queueSize;
        std::map<std::string, std::queue<float>> targetSystemQueue;

        void setUpContext();
        void updateRiskValues(RiskValues& context, const std::string& risk, const std::vector<std::string>& values);
        float pushQueueCalculateMean(std::string vitalSign, float data);
        void printAllRiskValues();
        void printRiskValues(const RiskValues& values, const std::string& vitalName);
        bool checkLowRisk(double data, const RiskValues sensorContext[], const int context);
        void checkContext(double risk, double data, const RiskValues context[], const char* contextName, int* targetContextCount);
        bool checkLowOrMidRisk(double data, const RiskValues sensorContext[], const int context);
        double calculateDisplacement(double data, const RiskValues sensorContext[], const int context);

        RiskValues heartRateContext[3];
        RiskValues oxigenationContext[3];
        RiskValues temperatureContext[3];
        RiskValues abpdContext[3];
        RiskValues abpsContext[3];
        RiskValues glucoseContext[3];
        TargetSystemData currentData;

        bool heartRateAvailable;
        bool oxigenationAvailable;
        bool temperatureAvailable;
        bool abpdAvailable;
        bool abpsAvailable;
        bool glucoseAvailable;
        void setUpAvailableSensors();

        bool setRisks(std::string vitalSign,float* lowRisk, float* MidRisk0, float* MidRisk1, float* highRisk0, float* highRisk1);

        //void monitor();
        void analyze();
        bool plan(const int targetContext);
        bool plan(const std::vector<int> repeatedContexts);
        void execute(const int targetContext);

};

#endif
#include "component/context_adaptation/ContextAdaptation.hpp"
#include "ros/ros.h"

ContextAdaptation::ContextAdaptation(int  &argc, char **argv, std::string name) : ROSComponent(argc, argv, name) {}

ContextAdaptation::~ContextAdaptation() {}

void ContextAdaptation::setUp() {
    srand(time(NULL));
    float freq;
    nh.getParam("context_frequency", freq);
    ROS_INFO("Setting Up");
    ROS_INFO("Freq = %f", freq);
    rosComponentDescriptor.setFreq(freq);
    setUpContext();
}

void ContextAdaptation::body() {
    //ros::Subscriber TargetSystemDataSub = nh.subscribe("TargetSystemData", 10, &ContextAdaptation::collect, this);
    ros::Rate loop_rate(rosComponentDescriptor.getFreq());
    while (ros::ok){
        ROS_INFO("Running");
        ros::spinOnce();
        loop_rate.sleep();            
    }   

    return;
}

void ContextAdaptation::setUpContext() {
    std::string s;
    std::vector<std::string> contexts = {"context0", "context1", "context2"};
    std::vector<std::string> risks = {"LowRisk", "MidRisk0", "MidRisk1", "HighRisk0", "HighRisk1"};

    for (int i = 0; i < contexts.size(); i++) {
        for (const auto& risk : risks) {
            std::string param_name = contexts[i] + "_heart_rate_" + risk;
            std::string s;
            nh.getParam(param_name, s);
            std::vector<std::string> values = bsn::utils::split(s, ',');

            if (risk == "LowRisk") {
                heart_rate_context[i].lowRisk[0] = std::stof(values[0]);
                heart_rate_context[i].lowRisk[1] = std::stof(values[1]);
                ROS_INFO("Setting up %s for %f, %f", param_name.c_str(), heart_rate_context[i].lowRisk[0], heart_rate_context[i].lowRisk[1]);
            } else if (risk == "MidRisk0") {
                heart_rate_context[i].midRisk0[0] = std::stof(values[0]);
                heart_rate_context[i].midRisk0[1] = std::stof(values[1]);
                ROS_INFO("Setting up %s for %f, %f", param_name.c_str(), heart_rate_context[i].midRisk0[0], heart_rate_context[i].midRisk0[1]);
            } else if (risk == "MidRisk1") {
                heart_rate_context[i].midRisk1[0] = std::stof(values[0]);
                heart_rate_context[i].midRisk1[1] = std::stof(values[1]);
                ROS_INFO("Setting up %s for %f, %f", param_name.c_str(), heart_rate_context[i].midRisk1[0], heart_rate_context[i].midRisk1[1]);
            } else if (risk == "HighRisk0") {
                heart_rate_context[i].highRisk0[0] = std::stof(values[0]);
                heart_rate_context[i].highRisk0[1] = std::stof(values[1]);
                ROS_INFO("Setting up %s for %f, %f", param_name.c_str(), heart_rate_context[i].highRisk0[0], heart_rate_context[i].highRisk0[1]);
            } else if (risk == "HighRisk1") {
                heart_rate_context[i].highRisk1[0] = std::stof(values[0]);
                heart_rate_context[i].highRisk1[1] = std::stof(values[1]);
                ROS_INFO("Setting up %s for %f, %f", param_name.c_str(), heart_rate_context[i].highRisk1[0], heart_rate_context[i].highRisk1[1]);
            }
        }
    }
}

void ContextAdaptation::tearDown() {
    ROS_INFO("Tearing down");
}

void ContextAdaptation::collect(const messages::TargetSystemData::ConstPtr& msg) {
    float heart_rate[2], spo2[2];
    heart_rate[0] = msg->ecg_data;
    spo2[0] = msg->oxi_data;

    heart_rate[1] = msg->ecg_risk;
    spo2[1] = msg->oxi_risk;

    float patient_risk = msg->patient_status;

    if(heart_rate[1] > 60){
        ROS_INFO("High heart_rate risk, maybe its another context\n risk = %.2f", heart_rate[1]);
        if(heart_rate[0]<90) {
            ROS_INFO("Maybe it is sleeping");
        }
        else {
            ROS_INFO("Maybe it is exercising");
        }
    }
}

bool ContextAdaptation::setRisks(std::string vitalSign, float* lowRisk, float* MidRisk0, float* MidRisk1, float* highRisk0, float* highRisk1) {
    ros::ServiceClient client = nh.serviceClient<services::PatientAdapt>("contextAdapt");
    services::PatientAdapt srv;
    srv.request.vitalSign = vitalSign;
    srv.request.lowRisk_floor   = lowRisk[0];
    srv.request.lowRisk_ceil    = lowRisk[1];
    srv.request.MidRisk0_floor  = MidRisk0[0];
    srv.request.MidRisk0_ceil   = MidRisk0[1];
    srv.request.MidRisk1_floor  = MidRisk1[0];
    srv.request.MidRisk1_ceil   = MidRisk1[1];
    srv.request.highRisk0_floor = highRisk0[0];
    srv.request.highRisk0_ceil  = highRisk0[1];
    srv.request.highRisk1_floor = highRisk1[0];
    srv.request.highRisk1_ceil  = highRisk1[1];

    if (client.call(srv)) {
        if(srv.response.set) {
            ROS_INFO("Params %s set successfully", vitalSign.c_str());
            return true;
        }
        else ROS_INFO("Could not write params");
    } else {
        ROS_INFO("Service is not answering");
    }
    return false;
}

void ContextAdaptation::monitor() {

}

void ContextAdaptation::analyze() {

}

void ContextAdaptation::plan() {

}

void ContextAdaptation::execute() {

}
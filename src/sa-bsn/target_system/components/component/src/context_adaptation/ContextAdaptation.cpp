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
    printAllRiskValues();
    currentContext = 0;
}

void ContextAdaptation::body() {
    ros::Subscriber TargetSystemDataSub = nh.subscribe("TargetSystemData", 10, &ContextAdaptation::monitor, this);
    ros::Rate loop_rate(rosComponentDescriptor.getFreq());
    while (ros::ok){
        ROS_INFO("Running");
        if(currentData.pending_analysis) {
            analyze();
            currentData.pending_analysis = false;
        }
        ros::spinOnce();
        loop_rate.sleep();            
    }   

    return;
}

void ContextAdaptation::setUpContext() {
    std::vector<std::string> contexts = {"context0", "context1", "context2"};
    std::vector<std::string> risks = {"LowRisk", "MidRisk0", "MidRisk1", "HighRisk0", "HighRisk1"};

    for (int i = 0; i < contexts.size(); i++) {
        for (const auto& risk : risks) {
            // Atualiza heartRateContext
            std::string heartRateParamName = contexts[i] + "_heart_rate_" + risk;
            std::string heartRateValuesStr;
            nh.getParam(heartRateParamName, heartRateValuesStr);
            std::vector<std::string> heartRateValues = bsn::utils::split(heartRateValuesStr, ',');            
            // Atualiza os valores de risco para heartRateContext
            updateRiskValues(heartRateContext[i], risk, heartRateValues);


            // Atualiza oxigenationContext
            std::string oxigenationParamName = contexts[i] + "_oxigenation_" + risk;
            std::string oxigenationValuesStr;
            nh.getParam(oxigenationParamName, oxigenationValuesStr);
            std::vector<std::string> oxigenationValues = bsn::utils::split(oxigenationValuesStr, ',');
            // Atualiza os valores de risco para oxigenationContext
            updateRiskValues(oxigenationContext[i], risk, oxigenationValues);

            // Atualiza temperatureContext
            std::string temperatureParamName = contexts[i] + "_temperature_" + risk;
            std::string temperatureValuesStr;
            nh.getParam(temperatureParamName, temperatureValuesStr);
            std::vector<std::string> temperatureValues = bsn::utils::split(temperatureValuesStr, ',');
            // Atualiza os valores de risco para temperatureContext
            updateRiskValues(temperatureContext[i], risk, temperatureValues);

            // Atualiza abpdContext
            std::string abpdParamName = contexts[i] + "_abpd_" + risk;
            std::string abpdValuesStr;
            nh.getParam(abpdParamName, abpdValuesStr);
            std::vector<std::string> abpdValues = bsn::utils::split(abpdValuesStr, ',');
            // Atualiza os valores de risco para abpdContext
            updateRiskValues(abpdContext[i], risk, abpdValues);

            // Atualiza abpsContext
            std::string abpsParamName = contexts[i] + "_abps_" + risk;
            ROS_INFO("abpsParamName = %s", abpsParamName.c_str());
            std::string abpsValuesStr;
            nh.getParam(abpsParamName, abpsValuesStr);
            std::vector<std::string> abpsValues = bsn::utils::split(abpsValuesStr, ',');
            // Atualiza os valores de risco para abpsContext
            updateRiskValues(abpsContext[i], risk, abpsValues);

            // Atualiza glucoseContext
            std::string glucoseParamName = contexts[i] + "_glucose_" + risk;
            std::string glucoseValuesStr;
            nh.getParam(glucoseParamName, glucoseValuesStr);
            std::vector<std::string> glucoseValues = bsn::utils::split(glucoseValuesStr, ',');
            // Atualiza os valores de risco para glucoseContext
            updateRiskValues(glucoseContext[i], risk, glucoseValues);
        }
    }
}

void ContextAdaptation::updateRiskValues(RiskValues& context, const std::string& risk, const std::vector<std::string>& values) {
    if (risk == "LowRisk") {
        context.lowRisk[0] = std::stof(values[0]);
        context.lowRisk[1] = std::stof(values[1]);
    } else if (risk == "MidRisk0") {
        context.midRisk0[0] = std::stof(values[0]);
        context.midRisk0[1] = std::stof(values[1]);
    } else if (risk == "MidRisk1") {
        context.midRisk1[0] = std::stof(values[0]);
        context.midRisk1[1] = std::stof(values[1]);
    } else if (risk == "HighRisk0") {
        context.highRisk0[0] = std::stof(values[0]);
        context.highRisk0[1] = std::stof(values[1]);
    } else if (risk == "HighRisk1") {
        context.highRisk1[0] = std::stof(values[0]);
        context.highRisk1[1] = std::stof(values[1]);
    }
}

void ContextAdaptation::printRiskValues(const RiskValues& values, const std::string& vitalName) {
    ROS_INFO("Risk values for %s:", vitalName.c_str());
    ROS_INFO("High Risk 0: [%.2f, %.2f]", values.highRisk0[0], values.highRisk0[1]);
    ROS_INFO("Mid Risk 0: [%.2f, %.2f]", values.midRisk0[0], values.midRisk0[1]);
    ROS_INFO("Low Risk: [%.2f, %.2f]", values.lowRisk[0], values.lowRisk[1]);
    ROS_INFO("Mid Risk 1: [%.2f, %.2f]", values.midRisk1[0], values.midRisk1[1]);
    ROS_INFO("High Risk 1: [%.2f, %.2f]", values.highRisk1[0], values.highRisk1[1]);
}

void ContextAdaptation::printAllRiskValues() {
    for (int i = 0; i < 3; ++i) {
        printRiskValues(oxigenationContext[i], "Oxigenation Context " + std::to_string(i));
        printRiskValues(heartRateContext[i], "Heart Rate Context " + std::to_string(i));
        printRiskValues(temperatureContext[i], "Temperature Context " + std::to_string(i));
        printRiskValues(abpdContext[i], "ABPD Context " + std::to_string(i));
        printRiskValues(abpsContext[i], "ABPS Context " + std::to_string(i));
        printRiskValues(glucoseContext[i], "Glucose Context " + std::to_string(i));
    }
}

void ContextAdaptation::tearDown() {
    ROS_INFO("Tearing down");
}

void ContextAdaptation::monitor(const messages::TargetSystemData::ConstPtr& msg) {
    currentData.trm_risk = msg->trm_risk;
    currentData.ecg_risk = msg->ecg_risk;
    currentData.oxi_risk = msg->oxi_risk;
    currentData.abps_risk = msg->abps_risk;
    currentData.abpd_risk = msg->abpd_risk;
    currentData.glc_risk = msg->glc_risk;
    currentData.trm_data = msg->trm_data;
    currentData.ecg_data = msg->ecg_data;
    currentData.oxi_data = msg->oxi_data;
    currentData.abps_data = msg->abps_data;
    currentData.abpd_data = msg->abpd_data;
    currentData.glc_data = msg->glc_data;
    currentData.patient_status = msg->patient_status;
    currentData.pending_analysis = true;

    ROS_INFO("Data collected");
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

void ContextAdaptation::analyze() {
    // Target context is the context is the context with the risk above certain value (60) and with its corresponding data in low risk
    // Array with the number of contexts that are low risk for each vital sign, the index is the target context
    int targetContextCount[3] = {0, 0, 0};
    checkContext(currentData.ecg_risk, currentData.ecg_data, heartRateContext, "Heart Rate", targetContextCount);
    checkContext(currentData.oxi_risk, currentData.oxi_data, oxigenationContext, "Oxigenation", targetContextCount);
    checkContext(currentData.trm_risk, currentData.trm_data, temperatureContext, "Temperature", targetContextCount);
    checkContext(currentData.abpd_risk, currentData.abpd_data, abpdContext, "ABPD", targetContextCount);
    checkContext(currentData.abps_risk, currentData.abps_data, abpsContext, "ABPS", targetContextCount);
    checkContext(currentData.glc_risk, currentData.glc_data, glucoseContext, "Glucose", targetContextCount);
    for(int i = 0; i < 3; i++) {
        ROS_INFO("Context %d is low risk for %d vital signs", i, targetContextCount[i]);
    }
    int targetContext = -1;
    int minCount = INT_MAX; // Inicializa com o maior valor possível

    for (int i = 0; i < 3; i++) {
        if (i != currentContext && targetContextCount[i] != 0 && targetContextCount[i] < minCount) {
            minCount = targetContextCount[i];
            targetContext = i;
        }
    }

    if(targetContext != -1) plan(targetContext);
    
}

void ContextAdaptation::checkContext(double risk, double data, const RiskValues context[], const char* contextName, int* targetContextCount) {
    if (risk > 60) {
        int targetContext = checkLowRisk(data, context);
        if (targetContext > 0 && targetContext != currentContext) {
            targetContextCount[targetContext]++;
            ROS_INFO("%s Data is low risk for context %d", contextName, targetContext);
        }
    }
}

// Verifica se está em baixo risco
int ContextAdaptation::checkLowRisk(double data, const RiskValues context[]) {
    for(int i = 0; i < 3; i++) {
        if (data >= context[i].lowRisk[0] && data <= context[i].lowRisk[1]) {
            return i;
        }
    }
    return -1;
}

void ContextAdaptation::plan(const int targetContext) {
    ROS_INFO("Selected target context: %d", targetContext);
    execute(targetContext);
}

void ContextAdaptation::execute(const int targetContext) {
    setRisks("oxigenation", oxigenationContext[targetContext].lowRisk, oxigenationContext[targetContext].midRisk0, oxigenationContext[targetContext].midRisk1, oxigenationContext[targetContext].highRisk0, oxigenationContext[targetContext].highRisk1);
    setRisks("heart_rate", heartRateContext[targetContext].lowRisk, heartRateContext[targetContext].midRisk0, heartRateContext[targetContext].midRisk1, heartRateContext[targetContext].highRisk0, heartRateContext[targetContext].highRisk1);
    setRisks("temperature", temperatureContext[targetContext].lowRisk, temperatureContext[targetContext].midRisk0, temperatureContext[targetContext].midRisk1, temperatureContext[targetContext].highRisk0, temperatureContext[targetContext].highRisk1);
    setRisks("abpd", abpdContext[targetContext].lowRisk, abpdContext[targetContext].midRisk0, abpdContext[targetContext].midRisk1, abpdContext[targetContext].highRisk0, abpdContext[targetContext].highRisk1);
    setRisks("abps", abpsContext[targetContext].lowRisk, abpsContext[targetContext].midRisk0, abpsContext[targetContext].midRisk1, abpsContext[targetContext].highRisk0, abpsContext[targetContext].highRisk1);
    setRisks("glucose", glucoseContext[targetContext].lowRisk, glucoseContext[targetContext].midRisk0, glucoseContext[targetContext].midRisk1, glucoseContext[targetContext].highRisk0, glucoseContext[targetContext].highRisk1);
    currentContext = targetContext;
}
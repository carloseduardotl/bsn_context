#include <PatientModule.hpp>

PatientModule::PatientModule(int  &argc, char **argv, std::string name) : ROSComponent(argc, argv, name) {}

PatientModule::~PatientModule() {}

void PatientModule::setUp() {
    srand(time(NULL));

    // TODO change Operation to static
    std::string vitalSigns;
    service = nh.advertiseService("getPatientData", &PatientModule::getPatientData, this);
    serviceAdapt = nh.advertiseService("contextAdapt", &PatientModule::setAdaptation, this);
    double aux;

    frequency = 1000;

    // Get what vital signs this module will simulate
    nh.getParam("vitalSigns", vitalSigns);

    // Removes white spaces from vitalSigns
    vitalSigns.erase(std::remove(vitalSigns.begin(), vitalSigns.end(),' '), vitalSigns.end());

    std::vector<std::string> splittedVitalSigns = bsn::utils::split(vitalSigns, ',');

    for (std::string s : splittedVitalSigns) {
        vitalSignsFrequencies[s] = 0;
        nh.getParam(s + "_Change", aux);
        vitalSignsChanges[s] = 1/aux;
        nh.getParam(s + "_Offset", vitalSignsOffsets[s]);
    }

    for (const std::string& s : splittedVitalSigns) {
        patientData[s] = configureDataGenerator(s);
    }

    rosComponentDescriptor.setFreq(frequency);
    
    period = 1/frequency;
}

bsn::generator::DataGenerator PatientModule::configureDataGenerator(const std::string& vitalSign) {
    srand(time(NULL));
    
    std::vector<std::string> t_probs;
    std::array<float, 25> transitions;
    std::array<bsn::range::Range,5> ranges;
    std::string s;
    ros::NodeHandle handle;

    for(uint32_t i = 0; i < transitions.size(); i++){
        for(uint32_t j = 0; j < 5; j++){
            handle.getParam(vitalSign + "_State" + std::to_string(j), s);
            t_probs = bsn::utils::split(s, ',');
            for(uint32_t k = 0; k < 5; k++){
                transitions[i++] = std::stod(t_probs[k]);
            }
        }
    }
    
    std::vector<std::string> lrs,mrs0,hrs0,mrs1,hrs1;

    handle.getParam(vitalSign + "_LowRisk", s);
    lrs = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_MidRisk0", s);
    mrs0 = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_HighRisk0", s);
    hrs0 = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_MidRisk1", s);
    mrs1 = bsn::utils::split(s, ',');
    handle.getParam(vitalSign + "_HighRisk1", s);
    hrs1 = bsn::utils::split(s, ',');

    ranges[0] = bsn::range::Range(std::stod(hrs0[0]), std::stod(hrs0[1]));
    ranges[1] = bsn::range::Range(std::stod(mrs0[0]), std::stod(mrs0[1]));
    ranges[2] = bsn::range::Range(std::stod(lrs[0]), std::stod(lrs[1]));
    ranges[3] = bsn::range::Range(std::stod(mrs1[0]), std::stod(mrs1[1]));
    ranges[4] = bsn::range::Range(std::stod(hrs1[0]), std::stod(hrs1[1]));

    bsn::generator::Markov markov(transitions, ranges, 2);
    bsn::generator::DataGenerator dataGenerator(markov);
    dataGenerator.setSeed();

    return dataGenerator;
}

void PatientModule::tearDown() {}

bool PatientModule::getPatientData(services::PatientData::Request &request, 
                                services::PatientData::Response &response) {
    
    response.data = patientData[request.vitalSign].getValue();
    
    ROS_INFO("Answered a request for %s's data.", request.vitalSign.c_str());

    return true;
}

void PatientModule::body() {
    for (auto &p : vitalSignsFrequencies) {
        
        if (p.second >= (vitalSignsChanges[p.first] + vitalSignsOffsets[p.first])) {
            patientData[p.first].nextState();
            p.second = vitalSignsOffsets[p.first];
            ROS_DEBUG("Transitioned %s's state", p.first.c_str());
        } else {
            p.second += period;
        }
        
    }
}


bool PatientModule::setAdaptation(services::PatientAdapt::Request &request, services::PatientAdapt::Response &response) {
    std::string vitalSign = request.vitalSign;
    ROS_INFO("Got an adaptation request for %s", vitalSign.c_str());
    if(!((vitalSign.compare("heart_rate") == 0) || (vitalSign.compare("oxigenation") == 0) || (vitalSign.compare("temperature") == 0) || (vitalSign.compare("abps") == 0) || (vitalSign.compare("abpd") == 0) || (vitalSign.compare("glucose") == 0))){
        ROS_INFO("But is not a valid vital sign");
        response.set = false;
        return true;
    }

    std::stringstream aux;
    aux.str("");
    aux << request.highRisk0_floor << "," << request.highRisk0_ceil;
    nh.setParam(vitalSign + "_HighRisk0", aux.str().c_str());
    ROS_INFO("%s_HighRisk0 = %s", vitalSign.c_str(), aux.str().c_str());

    aux.str("");
    aux << request.MidRisk0_floor << "," << request.MidRisk0_ceil;
    nh.setParam(vitalSign + "_MidRisk0", aux.str().c_str());
    ROS_INFO("%s_MidRisk0 = %s", vitalSign.c_str(), aux.str().c_str());

    aux.str("");
    aux << request.lowRisk_floor << "," << request.lowRisk_ceil;
    nh.setParam(vitalSign + "_LowRisk", aux.str().c_str());
    ROS_INFO("%s_LowRisk = %s", vitalSign.c_str(), aux.str().c_str());

    aux.str("");
    aux << request.MidRisk1_floor << "," << request.MidRisk1_ceil;
    nh.setParam(vitalSign + "_MidRisk1", aux.str().c_str());
    ROS_INFO("%s_MidRisk1 = %s", vitalSign.c_str(), aux.str().c_str());

    aux.str("");
    aux << request.highRisk1_floor << "," << request.highRisk1_ceil;
    nh.setParam(vitalSign + "_HighRisk1", aux.str().c_str());
    ROS_INFO("%s_HighRisk1 = %s", vitalSign.c_str(), aux.str().c_str());
    
    response.set = true;
    return true;
}

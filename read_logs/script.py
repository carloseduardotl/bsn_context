import re

confusion_matrix = {
    'TP': 0,
    'FP': 0,
    'TN': 0,
    'FN': 0
}

risk_threshold = 60

# Definir faixas de risco
risk_ranges = {
    # Risk values for oximeter (context 0)
    "context0_oxigenation_HighRisk0": (-1, -1),
    "context0_oxigenation_MidRisk0": (-1, -1),
    "context0_oxigenation_LowRisk": (65, 100),
    "context0_oxigenation_MidRisk1": (55, 65),
    "context0_oxigenation_HighRisk1": (0, 55),

    # Risk values for oximeter (context 1)
    "context1_oxigenation_HighRisk0": (-1, -1),
    "context1_oxigenation_MidRisk0": (-1, -1),
    "context1_oxigenation_LowRisk": (55, 100),
    "context1_oxigenation_MidRisk1": (45, 55),
    "context1_oxigenation_HighRisk1": (0, 45),

    # Risk values for oximeter (context 2)
    "context2_oxigenation_HighRisk0": (-1, -1),
    "context2_oxigenation_MidRisk0": (-1, -1),
    "context2_oxigenation_LowRisk": (85, 100),
    "context2_oxigenation_MidRisk1": (75, 85),
    "context2_oxigenation_HighRisk1": (0, 75),

    # Risk values for heart frequency (context 0)
    "context0_heart_rate_HighRisk0": (0, 70),
    "context0_heart_rate_MidRisk0": (70, 85),
    "context0_heart_rate_LowRisk": (85, 97),
    "context0_heart_rate_MidRisk1": (97, 115),
    "context0_heart_rate_HighRisk1": (115, 300),

    # Risk values for heart frequency (context 1)
    "context1_heart_rate_HighRisk0": (0, 40),
    "context1_heart_rate_MidRisk0": (40, 50),
    "context1_heart_rate_LowRisk": (50, 70),
    "context1_heart_rate_MidRisk1": (70, 80),
    "context1_heart_rate_HighRisk1": (80, 100),

    # Risk values for heart frequency (context 2)
    "context2_heart_rate_HighRisk0": (0, 80),
    "context2_heart_rate_MidRisk0": (80, 100),
    "context2_heart_rate_LowRisk": (100, 140),
    "context2_heart_rate_MidRisk1": (140, 160),
    "context2_heart_rate_HighRisk1": (160, 300),

    # Risk values for temperature (context 0)
    "context0_temperature_HighRisk0": (0, 31.99),
    "context0_temperature_MidRisk0": (32, 35.99),
    "context0_temperature_LowRisk": (36, 37.99),
    "context0_temperature_MidRisk1": (38, 40.99),
    "context0_temperature_HighRisk1": (41, 50),

    # Risk values for temperature (context 1)
    "context1_temperature_HighRisk0": (0, 29.99),
    "context1_temperature_MidRisk0": (30, 33.99),
    "context1_temperature_LowRisk": (34, 35.99),
    "context1_temperature_MidRisk1": (36, 38.99),
    "context1_temperature_HighRisk1": (39, 48),

    # Risk values for temperature (context 2)
    "context2_temperature_HighRisk0": (0, 32.99),
    "context2_temperature_MidRisk0": (33, 36.99),
    "context2_temperature_LowRisk": (37, 38.99),
    "context2_temperature_MidRisk1": (39, 41.99),
    "context2_temperature_HighRisk1": (42, 51),

    # Risk values for diastolic pressure (context 0)
    "context0_abpd_HighRisk0": (-1, -1),
    "context0_abpd_MidRisk0": (-1, -1),
    "context0_abpd_LowRisk": (0, 80),
    "context0_abpd_MidRisk1": (80, 90),
    "context0_abpd_HighRisk1": (90, 300),

    # Risk values for diastolic pressure (context 1)
    "context1_abpd_HighRisk0": (-1, -1),
    "context1_abpd_MidRisk0": (-1, -1),
    "context1_abpd_LowRisk": (0, 80),
    "context1_abpd_MidRisk1": (80, 90),
    "context1_abpd_HighRisk1": (90, 300),

    # Risk values for diastolic pressure (context 2)
    "context2_abpd_HighRisk0": (-1, -1),
    "context2_abpd_MidRisk0": (-1, -1),
    "context2_abpd_LowRisk": (0, 80),
    "context2_abpd_MidRisk1": (80, 90),
    "context2_abpd_HighRisk1": (90, 300),

    # Risk values for systolic pressure (context 0)
    "context0_abps_MidRisk0": (-1, -1),
    "context0_abps_HighRisk0": (-1, -1),
    "context0_abps_LowRisk": (0, 120),
    "context0_abps_MidRisk1": (120, 140),
    "context0_abps_HighRisk1": (140, 300),

    # Risk values for systolic pressure (context 1)
    "context1_abps_MidRisk0": (-1, -1),
    "context1_abps_HighRisk0": (-1, -1),
    "context1_abps_LowRisk": (0, 110),
    "context1_abps_MidRisk1": (110, 130),
    "context1_abps_HighRisk1": (130, 300),

    # Risk values for systolic pressure (context 2)
    "context2_abps_MidRisk0": (-1, -1),
    "context2_abps_HighRisk0": (-1, -1),
    "context2_abps_LowRisk": (0, 160),
    "context2_abps_MidRisk1": (160, 170),
    "context2_abps_HighRisk1": (170, 300),

    # Risk values for glucose (context 0)
    "context0_glucose_HighRisk0": (20, 39.99),
    "context0_glucose_MidRisk0": (40, 54.99),
    "context0_glucose_LowRisk": (55, 95.99),
    "context0_glucose_MidRisk1": (96, 119.99),
    "context0_glucose_HighRisk1": (120, 200),

    # Risk values for glucose (context 1)
    "context1_glucose_HighRisk0": (20, 30),
    "context1_glucose_MidRisk0": (30, 45),
    "context1_glucose_LowRisk": (45, 85),
    "context1_glucose_MidRisk1": (85, 105),
    "context1_glucose_HighRisk1": (105, 200),

    # Risk values for glucose (context 2)
    "context2_glucose_HighRisk0": (25, 35),
    "context2_glucose_MidRisk0": (35, 50),
    "context2_glucose_LowRisk": (50, 90),
    "context2_glucose_MidRisk1": (90, 110),
    "context2_glucose_HighRisk1": (110, 200)
}

def extract_current_data(line):
    # Expressão regular para extrair os valores da linha Current Data
    pattern = re.compile(r'trm_risk: ([\d.]+), ecg_risk: ([\d.]+), oxi_risk: ([\d.]+), abps_risk: ([\d.]+), abpd_risk: ([\d.]+), glc_risk: ([\d.]+), trm_data: ([\d.]+), ecg_data: ([\d.]+), oxi_data: ([\d.]+), abps_data: ([\d.]+), abpd_data: ([\d.]+), glc_data: ([\d.]+), patient_status: ([\d.]+)')
    match = pattern.search(line)
    if match:
        return {
            'trm_risk': float(match.group(1)),
            'ecg_risk': float(match.group(2)),
            'oxi_risk': float(match.group(3)),
            'abps_risk': float(match.group(4)),
            'abpd_risk': float(match.group(5)),
            'glc_risk': float(match.group(6)),
            'trm_data': float(match.group(7)),
            'ecg_data': float(match.group(8)),
            'oxi_data': float(match.group(9)),
            'abps_data': float(match.group(10)),
            'abpd_data': float(match.group(11)),
            'glc_data': float(match.group(12)),
            'patient_status': float(match.group(13))
        }
    return None

def extract_current_context(line):
    # Expressão regular para extrair o contexto atual
    pattern = re.compile(r'Current Context = (\d)')
    match = pattern.search(line)
    if match:
        return int(match.group(1))
    return None

def extract_one_context_from_line(line):
    # Expressão regular para extrair o contexto
    pattern = re.compile(r'context (\d)')
    match = pattern.search(line)
    if match:
        return int(match.group(1))
    return None

def extract_two_contexts_from_line(line):
    # Expressão regular para extrair todos os números
    pattern = re.compile(r'\b\d+\b')
    matches = pattern.findall(line)
    # Converte as correspondências para inteiros
    return [int(match) for match in matches]

def check_high_risk_is_low_risk(current_data, current_context):
    if current_data['trm_risk'] > risk_threshold:
        if is_value_in_risk_range_exclude_context(current_data['trm_data'], 'temperature_LowRisk', current_context):
            return True
    if current_data['oxi_risk'] > risk_threshold:
        if is_value_in_risk_range_exclude_context(current_data['oxi_data'], 'oxigenation_LowRisk', current_context):
            return True
    if(current_data['abpd_risk'] > risk_threshold):
        if is_value_in_risk_range_exclude_context(current_data['abpd_data'], 'abpd_LowRisk', current_context):
            return True
    if current_data['abps_risk'] > risk_threshold:
        if is_value_in_risk_range_exclude_context(current_data['abps_data'], 'abps_LowRisk', current_context):
            return True
    if current_data['glc_risk'] > risk_threshold:
        if is_value_in_risk_range_exclude_context(current_data['glc_data'], 'glucose_LowRisk', current_context):
            return True
    if current_data['ecg_risk'] > risk_threshold:
        if is_value_in_risk_range_exclude_context(current_data['ecg_data'], 'heart_rate_LowRisk', current_context):
            return True
    return False

def check_low_or_mid_risk(current_data, target_one_context):
    # Mapeamento dos sufixos de parâmetros para os sufixos de dados correspondentes
    parameter_to_data_suffix = {
        'temperature': 'trm_data',
        'oxigenation': 'oxi_data',
        'abpd': 'abpd_data',
        'abps': 'abps_data',
        'glucose': 'glc_data',
        'heart_rate': 'ecg_data'
    }
    risk_suffixes = ["LowRisk", "MidRisk0", "MidRisk1"]
    
    for param_suffix, data_suffix in parameter_to_data_suffix.items():
        for risk_suffix in risk_suffixes:
            parameter_key = f"context{target_one_context}_{param_suffix}_{risk_suffix}"
            # Verifica se a chave existe no dicionário current_data antes de chamar a função
            if data_suffix in current_data:
                if not is_value_in_risk_range(current_data[data_suffix], parameter_key):
                    return False
    return True

def is_value_in_risk_range(value, parameter):
    # ex: parameter = 'context0_glucose_LowRisk'
    if parameter in risk_ranges:
        low, high = risk_ranges[parameter]
        if low <= value <= high:
            #print(low, value, high)
            return True
    return False

def is_value_in_risk_range_exclude_context(value, parameter, current_context):
    # ex: parameter = 'glucose_LowRisk'
    for context in range(3):  # Supondo que temos 3 contextos (0, 1, 2)
        if context != current_context:
            parameter_key = f"context{context}_{parameter}" # ex: context0_glucose_LowRisk
            if parameter_key in risk_ranges:
                low, high = risk_ranges[parameter_key]
                if low <= value <= high:
                    #print(f"Valor dentro da faixa de risco: {parameter_key}")
                    #print(low, value, high)
                    return True
    #print(f"{parameter} não encontrado em faixas de risco fora do contexto atual")
    return False

def process_log_file(input_file):
    with open(input_file, 'r') as infile:
        lines = infile.readlines()

    current_data_list = []
    for line in lines:
        if 'Current Context = ' in line:
            current_context = extract_current_context(line)
        if 'Current Data' in line:
            current_data = extract_current_data(line)
        if 'No target context found' in line:
            if check_high_risk_is_low_risk(current_data, current_context):
                confusion_matrix['FN'] += 1
            else:
                confusion_matrix['TN'] += 1
        if 'Current data is not low or mid risk for context' in line:
            target_one_context = extract_one_context_from_line(line)
            if check_low_or_mid_risk(current_data, target_one_context):
                confusion_matrix['FN'] += 1
            else:
                confusion_matrix['TN'] += 1
        if 'Current data is not low or mid risk for any of the contexts' in line:
            target_one_context, target_two_context = extract_two_contexts_from_line(line)
            if check_low_or_mid_risk(current_data, target_one_context) or check_low_or_mid_risk(current_data, target_two_context):
                confusion_matrix['FN'] += 1
            else:
                confusion_matrix['TN'] += 1

            

if __name__ == "__main__":
    input_file = 'processed_rosout.log'
    process_log_file(input_file)
    print(confusion_matrix)
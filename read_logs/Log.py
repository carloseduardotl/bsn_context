def filter_and_process_lines(input_file, output_file):
    with open(input_file, 'r') as infile:
        lines = infile.readlines()

    # Filtrar as linhas que contêm '/context_adaptation'
    context_adaptation_lines = [line for line in lines if '/context_adaptation' in line]

    # Processar as linhas para remover tudo antes dos nomes em parênteses e '[topics: /rosout]'
    processed_lines = []
    for line in context_adaptation_lines:
        # Remover '[topics: /rosout]' se presente
        line = line.replace('[topics: /rosout]', '')
        
        # Encontrar a posição do primeiro parêntese
        pos = line.find('(')
        if pos != -1:
            processed_lines.append(line[pos:])
        else:
            processed_lines.append(line)

    with open(output_file, 'w') as outfile:
        outfile.writelines(processed_lines)

if __name__ == "__main__":
    input_file = 'rosout.log'
    output_file = 'processed_rosout.log'
    filter_and_process_lines(input_file, output_file)
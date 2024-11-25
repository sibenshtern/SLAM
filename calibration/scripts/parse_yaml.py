def get_parameters_from_yaml(file_name, parameters):
    def str_to_list(string: str) -> list:
        return list(map(float, string.strip('[]').split(', ')))


    with open(file_name, mode='r') as file:
        data = {}

        current_cam = None
        for line in file:
            if line.startswith("cam"):
                current_cam = line.strip(':\n')
                data[current_cam] = {}

            fixed_line = [obj.strip() for obj in line.strip().split(': ')]
            if fixed_line[0] in parameters:
                value = fixed_line[1]
                if fixed_line[1].startswith('['):
                    value = str_to_list(fixed_line[1])

                data[current_cam][fixed_line[0]] = value

    return data

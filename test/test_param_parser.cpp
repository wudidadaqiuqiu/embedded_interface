#include <rclcpp/rclcpp.hpp>
#include <rcl_yaml_param_parser/parser.h>
#include <iostream>
#include <memory>
#include <string>

void parse_yaml_file(const std::string &file_path) {
    rcl_params_t *params = rcl_yaml_node_struct_init(nullptr);
    if (!rcl_parse_yaml_file(file_path.c_str(), params)) {
        std::cerr << "Failed to parse YAML file: " << file_path << std::endl;
        rcl_yaml_node_struct_fini(params);
        return;
    }

    // 打印所有节点和参数
    for (size_t i = 0; i < params->num_nodes; ++i) {
        std::cout << "Node: " << params->node_names[i] << std::endl;
        for (size_t j = 0; j < params->params[i].num_params; ++j) {
            auto param_name = params->params[i].parameter_names[j];
            auto param_value = &params->params[i].parameter_values[j];
            std::cout << "  " << param_name << " = ";
            if (param_value->type == RCL_PARAM_BOOL) {
                std::cout << (param_value->bool_value ? "true" : "false");
            } else if (param_value->type == RCL_PARAM_INT) {
                std::cout << param_value->integer_value;
            } else if (param_value->type == RCL_PARAM_DOUBLE) {
                std::cout << param_value->double_value;
            } else if (param_value->type == RCL_PARAM_STRING) {
                std::cout << param_value->string_value;
            }
            std::cout << std::endl;
        }
    }

    rcl_yaml_node_struct_fini(params);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <params_file.yaml>" << std::endl;
        return 1;
    }

    parse_yaml_file(argv[1]);
    return 0;
}

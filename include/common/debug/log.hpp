#pragma once

#ifdef ROS2_DEBUG
#include "rclcpp/rclcpp.hpp"
#define LOG_DEBUG(CONDITION, ...) do {if(CONDITION) RCLCPP_DEBUG(__VA_ARGS__);} while(0)
#define LOG_INFO(CONDITION, ...) do {if(CONDITION) RCLCPP_INFO(__VA_ARGS__);} while(0)
#define LOG_WARN(CONDITION, ...) do {if(CONDITION) RCLCPP_WARN(__VA_ARGS__);} while(0)
#define LOG_ERROR(CONDITION, ...) do {if(CONDITION) RCLCPP_ERROR(__VA_ARGS__);} while(0)
#define LOG_FATAL(CONDITION, ...) do {if(CONDITION) RCLCPP_FATAL(__VA_ARGS__);} while(0)

#else
#include <cstdio>
#define  LOG_DEBUG(CONDITION, ...) do {if(CONDITION){ printf(__VA_ARGS__); printf("\n"); }} while(0)
#define LOG_INFO(CONDITION, ...) do {if(CONDITION) { printf("\033[32m"); printf(__VA_ARGS__); printf("\033[0m\n"); }} while(0)
#define LOG_WARN(CONDITION, ...) do {if(CONDITION) { printf("\033[33m"); printf(__VA_ARGS__); printf("\033[0m\n"); }} while(0)
#define LOG_ERROR(CONDITION, ...) do {if(CONDITION) { printf("\033[31m"); printf(__VA_ARGS__); printf("\033[0m\n");}} while(0)
#define LOG_FATAL(CONDITION, ...) do {if(CONDITION) { printf("\033[31;7m"); printf(__VA_ARGS__);printf("\033[0m\n");}} while(0)
#endif
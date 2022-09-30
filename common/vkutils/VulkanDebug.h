#pragma once
#include "vulkan/vulkan.h"

#include <string>
#include <vector>
#ifdef __ANDROID__
#include "VulkanAndroid.h"
#endif
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

namespace vks
{
	namespace debug
	{
		// Default validation layers
		extern int validationLayerCount;
		extern const char *validationLayerNames[];

		// Default debug callback
		VKAPI_ATTR VkBool32 VKAPI_CALL messageCallback(
			VkDebugReportFlagsEXT flags,
			VkDebugReportObjectTypeEXT objType,
			uint64_t srcObject,
			size_t location,
			int32_t msgCode,
			const char* pLayerPrefix,
			const char* pMsg,
			void* pUserData);

		// Load debug function pointers and set debug callback
		// if callBack is NULL, default message callback will be used
		void setupDebugging(
			VkInstance instance,
			VkDebugReportFlagsEXT flags,
			VkDebugReportCallbackEXT callBack);
		// Clear debug callback
		void freeDebugCallback(VkInstance instance);
	}
}

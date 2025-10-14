// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:action/Jtraj.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/action/jtraj.h"


#ifndef INTERFACES__ACTION__DETAIL__JTRAJ__STRUCT_H_
#define INTERFACES__ACTION__DETAIL__JTRAJ__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'qf'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in action/Jtraj in the package interfaces.
typedef struct interfaces__action__Jtraj_Goal
{
  rosidl_runtime_c__double__Sequence qf;
  double tf;
} interfaces__action__Jtraj_Goal;

// Struct for a sequence of interfaces__action__Jtraj_Goal.
typedef struct interfaces__action__Jtraj_Goal__Sequence
{
  interfaces__action__Jtraj_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__Jtraj_Goal__Sequence;

// Constants defined in the message

/// Struct defined in action/Jtraj in the package interfaces.
typedef struct interfaces__action__Jtraj_Result
{
  bool success;
} interfaces__action__Jtraj_Result;

// Struct for a sequence of interfaces__action__Jtraj_Result.
typedef struct interfaces__action__Jtraj_Result__Sequence
{
  interfaces__action__Jtraj_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__Jtraj_Result__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'q_now'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in action/Jtraj in the package interfaces.
typedef struct interfaces__action__Jtraj_Feedback
{
  rosidl_runtime_c__double__Sequence q_now;
} interfaces__action__Jtraj_Feedback;

// Struct for a sequence of interfaces__action__Jtraj_Feedback.
typedef struct interfaces__action__Jtraj_Feedback__Sequence
{
  interfaces__action__Jtraj_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__Jtraj_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "interfaces/action/detail/jtraj__struct.h"

/// Struct defined in action/Jtraj in the package interfaces.
typedef struct interfaces__action__Jtraj_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  interfaces__action__Jtraj_Goal goal;
} interfaces__action__Jtraj_SendGoal_Request;

// Struct for a sequence of interfaces__action__Jtraj_SendGoal_Request.
typedef struct interfaces__action__Jtraj_SendGoal_Request__Sequence
{
  interfaces__action__Jtraj_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__Jtraj_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Jtraj in the package interfaces.
typedef struct interfaces__action__Jtraj_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} interfaces__action__Jtraj_SendGoal_Response;

// Struct for a sequence of interfaces__action__Jtraj_SendGoal_Response.
typedef struct interfaces__action__Jtraj_SendGoal_Response__Sequence
{
  interfaces__action__Jtraj_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__Jtraj_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  interfaces__action__Jtraj_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  interfaces__action__Jtraj_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/Jtraj in the package interfaces.
typedef struct interfaces__action__Jtraj_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  interfaces__action__Jtraj_SendGoal_Request__Sequence request;
  interfaces__action__Jtraj_SendGoal_Response__Sequence response;
} interfaces__action__Jtraj_SendGoal_Event;

// Struct for a sequence of interfaces__action__Jtraj_SendGoal_Event.
typedef struct interfaces__action__Jtraj_SendGoal_Event__Sequence
{
  interfaces__action__Jtraj_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__Jtraj_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Jtraj in the package interfaces.
typedef struct interfaces__action__Jtraj_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} interfaces__action__Jtraj_GetResult_Request;

// Struct for a sequence of interfaces__action__Jtraj_GetResult_Request.
typedef struct interfaces__action__Jtraj_GetResult_Request__Sequence
{
  interfaces__action__Jtraj_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__Jtraj_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "interfaces/action/detail/jtraj__struct.h"

/// Struct defined in action/Jtraj in the package interfaces.
typedef struct interfaces__action__Jtraj_GetResult_Response
{
  int8_t status;
  interfaces__action__Jtraj_Result result;
} interfaces__action__Jtraj_GetResult_Response;

// Struct for a sequence of interfaces__action__Jtraj_GetResult_Response.
typedef struct interfaces__action__Jtraj_GetResult_Response__Sequence
{
  interfaces__action__Jtraj_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__Jtraj_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  interfaces__action__Jtraj_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  interfaces__action__Jtraj_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/Jtraj in the package interfaces.
typedef struct interfaces__action__Jtraj_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  interfaces__action__Jtraj_GetResult_Request__Sequence request;
  interfaces__action__Jtraj_GetResult_Response__Sequence response;
} interfaces__action__Jtraj_GetResult_Event;

// Struct for a sequence of interfaces__action__Jtraj_GetResult_Event.
typedef struct interfaces__action__Jtraj_GetResult_Event__Sequence
{
  interfaces__action__Jtraj_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__Jtraj_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "interfaces/action/detail/jtraj__struct.h"

/// Struct defined in action/Jtraj in the package interfaces.
typedef struct interfaces__action__Jtraj_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  interfaces__action__Jtraj_Feedback feedback;
} interfaces__action__Jtraj_FeedbackMessage;

// Struct for a sequence of interfaces__action__Jtraj_FeedbackMessage.
typedef struct interfaces__action__Jtraj_FeedbackMessage__Sequence
{
  interfaces__action__Jtraj_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__action__Jtraj_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__ACTION__DETAIL__JTRAJ__STRUCT_H_

﻿// ---------------------------------------------------------------------
// THIS FILE IS AUTO-GENERATED BY BEHAVIAC DESIGNER, SO PLEASE DON'T MODIFY IT BY YOURSELF!
// ---------------------------------------------------------------------

#ifndef _BEHAVIAC_CUSTOMIZED_TYPES_H_
#define _BEHAVIAC_CUSTOMIZED_TYPES_H_

#include "behaviac/agent/agent.h"

// -------------------
// Customized enums
// -------------------

enum TaskType
{
	SetTargetType,
	TurnBack,
	MoveTo,
};

DECLARE_BEHAVIAC_ENUM_EX(TaskType, TaskType);
BEHAVIAC_DECLARE_TYPE_VECTOR_HANDLER(TaskType);


#endif // _BEHAVIAC_CUSTOMIZED_TYPES_H_

#pragma once

#include "../engine/core/bodyid.hpp"

inline BodyID g_selected_body_id = 0;

inline void SetSelectedBodyId(BodyID id)
{
	g_selected_body_id = id;
}

inline BodyID GetSelectedBodyId()
{
	return g_selected_body_id;
}


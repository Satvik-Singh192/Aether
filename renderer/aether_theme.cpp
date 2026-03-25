#include "aether_theme.hpp"

#include <filesystem>

#include "imgui.h"

void LoadAetherFont()
{
    ImGuiIO &io = ImGui::GetIO();
    const char *segoePath = "C:/Windows/Fonts/segoeui.ttf";

    if (std::filesystem::exists(segoePath))
    {
        if (ImFont *font = io.Fonts->AddFontFromFileTTF(segoePath, 18.0f))
        {
            io.FontDefault = font;
            return;
        }
    }

    io.FontDefault = io.Fonts->AddFontDefault();
}

void ApplyAetherTheme()
{
    ImGuiStyle &style = ImGui::GetStyle();
    ImVec4 *colors = style.Colors;

    style.WindowRounding = 8.0f;
    style.FrameRounding = 6.0f;
    style.PopupRounding = 6.0f;
    style.GrabRounding = 6.0f;
    style.TabRounding = 6.0f;
    style.WindowBorderSize = 0.0f;
    style.FrameBorderSize = 0.0f;

    style.WindowPadding = ImVec2(16.0f, 16.0f);
    style.FramePadding = ImVec2(12.0f, 6.0f);
    style.ItemSpacing = ImVec2(10.0f, 10.0f);
    style.ItemInnerSpacing = ImVec2(8.0f, 8.0f);

    colors[ImGuiCol_WindowBg] = ImVec4(0.10f, 0.10f, 0.12f, 0.98f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.12f, 0.12f, 0.14f, 1.00f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.10f, 0.10f, 0.12f, 0.98f);

    colors[ImGuiCol_FrameBg] = ImVec4(0.06f, 0.06f, 0.07f, 1.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.15f, 0.16f, 0.19f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.20f, 0.22f, 0.25f, 1.00f);

    colors[ImGuiCol_Header] = ImVec4(0.15f, 0.16f, 0.19f, 1.00f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.20f, 0.22f, 0.25f, 1.00f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.24f, 0.26f, 0.30f, 1.00f);

    colors[ImGuiCol_Button] = ImVec4(0.15f, 0.16f, 0.19f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.22f, 0.45f, 0.85f, 0.90f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.18f, 0.38f, 0.75f, 1.00f);

    colors[ImGuiCol_SliderGrab] = ImVec4(0.22f, 0.45f, 0.85f, 0.90f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.18f, 0.38f, 0.75f, 1.00f);

    colors[ImGuiCol_CheckMark] = ImVec4(0.22f, 0.45f, 0.85f, 1.00f);
    colors[ImGuiCol_Text] = ImVec4(0.90f, 0.90f, 0.92f, 1.00f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.50f, 0.52f, 1.00f);

    colors[ImGuiCol_MenuBarBg] = ImVec4(0.12f, 0.12f, 0.14f, 1.00f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.12f, 0.12f, 0.14f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.15f, 0.16f, 0.19f, 1.00f);
    colors[ImGuiCol_Separator] = ImVec4(0.20f, 0.22f, 0.25f, 1.00f);
    colors[ImGuiCol_Tab] = ImVec4(0.15f, 0.16f, 0.19f, 1.00f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.22f, 0.45f, 0.85f, 0.90f);
    colors[ImGuiCol_TabActive] = ImVec4(0.18f, 0.38f, 0.75f, 1.00f);
}

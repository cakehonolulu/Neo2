#include <frontends/imgui/imgui_neo2.h>

int main(int argc, char **argv)
{
    ImGui_Neo2 neo2;
    neo2.init();
    neo2.run(argc, argv);
    return 0;
}

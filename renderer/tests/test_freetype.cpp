#include <ft2build.h>
#include FT_FREETYPE_H

int main()
{
    FT_Library ft;

    if(FT_Init_FreeType(&ft)) {
        fprintf(stderr, "Could not init freetype library\n");
        return 1;
    }
}

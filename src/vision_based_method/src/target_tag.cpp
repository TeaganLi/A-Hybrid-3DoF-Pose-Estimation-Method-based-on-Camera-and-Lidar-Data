#include <stdlib.h>
#include "apriltag.h"

apriltag_family_t *target_tag_create()
{
   apriltag_family_t *tf = (apriltag_family_t*)calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("targetTag");
   tf->black_border = 1;
   tf->d = 6;
   tf->h = 10;
   tf->ncodes = 1;
   tf->codes = (uint64_t*)calloc(1, sizeof(uint64_t));
   tf->codes[0] = 0x00438c300UL;
   return tf;
}

void target_tag_destroy(apriltag_family_t *tf)
{
   free(tf->name);
   free(tf->codes);
   free(tf);
}

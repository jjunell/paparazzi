
#include <image.h>


 void sobel_edge_filter(struct img_struct *input,struct img_struct *output);
void blur_filter(struct img_struct *input,struct img_struct *output);
 void image_difference(struct img_struct *input,struct img_struct *input_prev,struct img_struct *output,int count);

# the x_min and y_min corresponds to the upper left corner (not the center)
uint32 x
uint32 y

# the width and height of the bbox
uint32 width
uint32 height

# cols(width) and rows(height) of the original image
uint32 cols 
uint32 rows

bool occluded # default is false

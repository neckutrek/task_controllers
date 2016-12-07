#includes frame name of the map
Header header
#size of the map in number of cells
int32 XSize
int32 YSize
int32 ZSize
#resolution of each cell
float64 resolution
#maximum value of fusing weights
float64 Wmax
#truncation region min and max
float64 Dmax
float64 Dmin
#the actual map data. size is x*y*z*2
float32[] grid

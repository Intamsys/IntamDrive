#ifndef LIBGEN_H
#define LIBGEN_H

#include <string.h>

//self made function
static char * dirname(char * str)
  {
    static  char dir[256];
    if(strchr(str,'/'))
        strncpy(dir,str,strlen(str)-strlen(strrchr(str,'/')));
    else if(strchr(str,'\\'))
        strncpy(dir,str,strlen(str)-strlen(strrchr(str,'\\')));
    else
        return str;
    return dir;
  }

static char *  basename(char * str){
    static  char base[256];

    if(strchr(str,'/'))
        strncpy(base,strrchr(str,'/')+1,strlen(strrchr(str,'/'))-1);
    else if(strchr(str,'\\'))
        strncpy(base,strrchr(str,'\\')+1,strlen(strrchr(str,'\\'))-1);
    else
        return str;

    return base;
}

#endif /* _YVALS */


#include <stdint.h>

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#define BUFFSIZE 65536

int
ParseVisionFocus(int length, char * infile)
{
     char * ptr;
     char * ptr2;
     int count;
     int i;
     char localcopy[BUFFSIZE];
     int parsedDistance[65536];
     int minDistance = 0;
     int maxDistance = 65534;
     int tDone;
     int lowlimit;
     int highlimit;
     char testStr[1024];
     char shd[1024];
     char number1[1024];
     char number2[1024];
     char tmpline[256];
     int seekdir;
     int defaultdistance;
     int i1, i2;
     int FileLength;
     int itest;     
     int tmplen;

     for ( i=0; i<65536; i++ ) {
         parsedDistance[i] = 0;
     }

     if ( length >= BUFFSIZE ) { return(-1) ; }
     FileLength = length;

     for ( i=0; i<BUFFSIZE; i++ ) {
         localcopy[i] = 0;
     }
     strncpy( localcopy, infile, FileLength );

     // change all alpha chars to lower case
     for ( i=0; i<FileLength; i++ ) {
         if ( isupper( localcopy[i] ) ) {
              localcopy[i] = tolower( localcopy[i] );
         }
     }

     ptr = &(localcopy[0]);
     count = 0;
     tDone = 0;
     while ( count < FileLength && *ptr != 0 )\
       {
	 // no periods allowed
	 if ( *ptr == 0x2E ) {
	   fprintf( stderr, "ParseVisionFocus error: period at %d\n", count);
	   return( -1 );
	 }

	 // at tDone = 0, look for lowlimit
	 strcpy( testStr, "lowlimit" );
	 if ( strncmp( ptr, testStr, strlen(testStr )) == 0 && tDone == 0 ) {
	   strncpy( tmpline, ptr, 255 );
	   for ( i = 0; i < 255; i++ ) {
	     if ( tmpline[i] == 0x0a || tmpline[i] == 0x0d ) {
	       tmpline[i] = 0;
	     }
	   }
	   itest = sscanf( tmpline, "lowlimit = %s", number1 );
	   if ( itest != 1 ) { 
             fprintf( stderr 
		      , "ParseVisionFocus error: %s sscanf at %d  itest %d\n"
		      , testStr
		      , count
		      , itest
		      );
             return( -1 );
	   }
	   tmplen = strlen( number1 ); 
	   for ( i=0; i < tmplen; i++ ) {
	     if ( ! isdigit( number1[i] ) ) {
	       fprintf( stderr 
			, "ParseVisionFocus error: %s isdigit %d  number1 %c \n"
			, testStr
			, i
			, number1[i]
			);
	       return( -1 );
	     }
	   }
	   sscanf( number1, "%d", &lowlimit );
	   tDone = 1;
	 } 
	 
	 // at tDone = 1, look for highlimit
	 strcpy( testStr, "highlimit" );
	 if ( strncmp( ptr, testStr, strlen(testStr )) == 0 && tDone == 1 ) {
	   strncpy( tmpline, ptr, 255 );
	   for ( i = 0; i < 255; i++ ) {
	     if ( tmpline[i] == 0x0a || tmpline[i] == 0x0d ) {
	       tmpline[i] = 0;
	     }
	   }
	   itest = sscanf( tmpline, "highlimit = %s", number1 );
	   if ( itest != 1 ) { 
             fprintf( stderr 
		      , "ParseVisionFocus error: %s sscanf at %d  itest %d\n"
		      , testStr
		      , count
		      , itest
		      );
             return( -1 ); 
	   }
          tmplen = strlen( number1 ); 
          for ( i=0; i < tmplen; i++ ) {
	    if ( ! isdigit( number1[i] ) ) {
	      fprintf( stderr 
		       , "ParseVisionFocus error: %s isdigit %d  number1 %c \n"
		       , testStr
		       , i
		       , number1[i]
		       );
	      return( -1 );
	    }
          }
          sscanf( number1, "%d", &highlimit );
          tDone = 2;
	 } 
	 
	 // at tDone = 2, look for seekhomedirection 
	 strcpy( testStr, "seekhomedirection" );
	 if ( strncmp( ptr, testStr, strlen(testStr )) == 0 && tDone == 2 ) {
	   strncpy( tmpline, ptr, 255 );
	   for ( i = 0; i < 255; i++ ) {
	     if ( tmpline[i] == 0x0a || tmpline[i] == 0x0d ) {
	       tmpline[i] = 0;
	     }
          }
	   itest = sscanf( tmpline, "seekhomedirection = %s", shd );
	   if ( itest != 1 ) { 
             fprintf( stderr 
		      , "ParseVisionFocus error: %s sscanf at %d  itest %d\n"
		      , testStr
		      , count
		      , itest
		      );
             return( -1 );
	   }
	   if ( strncmp( shd, "cw", 2 ) == 0 ) {
             seekdir = 1;
	   } else if ( strncmp( shd, "ccw", 3 ) == 0 ) {
             seekdir = 2;
	   }
	   else
	     {
	       fprintf(stderr, 
		       "ParseVisionFocus error: %s sscanf at %d  not CW or CCW\n",
		       testStr, count);
	       return(-1);
	     }
	   tDone = 3;
	 } 
	 
	 // at tDone = 3, look for highlimit
	 strcpy( testStr, "defaultdistance");
	 if ( strncmp(ptr, testStr, strlen(testStr )) == 0 && tDone == 3 ) {
	   strncpy( tmpline, ptr, 255 );
	   for ( i = 0; i < 255; i++)
	     {
	       if ((tmpline[i] == 0x0a) || (tmpline[i] == 0x0d))
		 tmpline[i] = 0;
	     }
	   itest = sscanf( tmpline, "defaultdistance = %s", number1 );
	   if (itest != 1)
	     { 
	       fprintf(stderr, 
		       "ParseVisionFocus error: %s sscanf at %d  itest %d seekdir %d\n",
		       testStr, count, itest, seekdir);
             return(-1); 
	   }
	   tmplen = strlen( number1 ); 
	   for ( i=0; i < tmplen; i++ ) {
	     if ( ! isdigit( number1[i] ) ) {
	       fprintf( stderr 
			, "ParseVisionFocus error: %s isdigit %d  number1 %c \n"
			, testStr
			, i
			, number1[i]
			);
	       return( -1 );
	     }
	   }
	   sscanf( number1, "%d", &defaultdistance );
	   tDone = 4;
	 } 
	 
	 if ( tDone>=4 && ptr[0]==0x0D && ptr[1]==0x0A && (count+2)<length ) {
	   tDone++;
	   ptr2 = &(ptr[2]);
	   strncpy( tmpline, ptr2, 255 );
	   for ( i = 0; i < 255; i++ ) {
	     if ( tmpline[i] == 0x0a || tmpline[i] == 0x0d ) {
	       tmpline[i] = 0;
	     }
	   }
	   itest = sscanf( tmpline, "%s = %s", number1, number2 );
	   if ( itest != 2 ) { 
             fprintf( stderr 
		      , "ParseVisionFocus error: distance sscanf at %d  itest %d\n"
		      , count
		      , itest
		      );
             return( -1 ); 

	   } else {
	     
             tmplen = strlen( number1 ); 
             for ( i=0; i < tmplen; i++ ) {
	       if ( ! isdigit( number1[i] ) ) {
		 fprintf( stderr 
			  ,"ParseVisionFocus error: %s isdigit %d  number1 %c \n"
			  , testStr
			  , i
			  , number1[i]
			  );
		 return( -1 );
	       }
             }
             sscanf( number1, "%d", &i1 );
	     
             tmplen = strlen( number2 ); 
             for ( i=0; i < tmplen; i++ ) {
	       if ( ! isdigit( number2[i] ) ) {
		 fprintf( stderr 
			  ,"ParseVisionFocus error: %s isdigit %d  number2 %c \n"
			  , testStr
			  , i
			  , number2[i]
			  );
		 return( -1 );
	       }
             }
             sscanf( number2, "%d", &i2 );
	     
             if ( i2 < lowlimit ) {
	       fprintf(stderr, "ParseVisionFocus error: ");
	       fprintf(stderr, "sscanf at %d ", count);
	       fprintf(stderr, " i2 %d ", i2);
	       fprintf(stderr, " lowlimit %d \n", lowlimit);
	       return( -1 ); 
             } 

             if ( i2 > highlimit ) {
	       fprintf(stderr, "ParseVisionFocus error: ");
	       fprintf(stderr, "sscanf at %d ", count);
	       fprintf(stderr, " i2 %d ", i2);
	       fprintf(stderr, " highlimit %d \n", highlimit);
	       return( -1 ); 
             } 

             if ( i1 < minDistance ) {
	       fprintf(stderr, "ParseVisionFocus error: ");
	       fprintf(stderr, "sscanf at %d ", count);
	       fprintf(stderr, " i1 %d ", i1);
	       fprintf(stderr, " minDistance %d \n", minDistance);
	       return( -1 ); 
             }
             if ( i1 > maxDistance ) {
	       fprintf(stderr, "ParseVisionFocus error: ");
	       fprintf(stderr, "sscanf at %d ", count);
	       fprintf(stderr, " i1 %d ", i1);
	       fprintf(stderr, " maxDistance %d \n", maxDistance);
	       return( -1 ); 
             }
             if ( parsedDistance[i1] == 1 ) {
                 fprintf(stderr, "ParseVisionFocus error: ");
                 fprintf(stderr, "sscanf at %d ", count);
                 fprintf(stderr, " i1 %d ", i1);
                 fprintf(stderr, " duplicate parsedDistance \n");
                 return( -1 ); 
             }
             parsedDistance[i1] = 1;
	   }
	 }
	 
	 ptr++;
	 count++;
       }
     
     if ( tDone < 4 ) {
       fprintf( stderr, "ParseVisionFocus error: tDone %d\n", tDone );
       return( -1 );
     }
     return ( 0 );
}

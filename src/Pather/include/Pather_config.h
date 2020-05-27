#pragma once

#if defined( _WIN32 ) && defined( Pather_SHARED_LIBRARY )
#ifdef Pather_EXPORTS
#define PATHER_EXPORTS __declspec( dllexport )
#else
#define PATHER_EXPORTS __declspec( dllimport )
#endif
#else
#define PATHER_EXPORTS
#endif
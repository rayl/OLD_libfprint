AC_INIT([libfprint], [0.1.0-pre2])
AM_INIT_AUTOMAKE
AC_CONFIG_MACRO_DIR([m4])
AC_CONFIG_SRCDIR([libfprint/core.c])
AM_CONFIG_HEADER([config.h])

AC_PREREQ([2.50])
AC_PROG_CC
AC_PROG_LIBTOOL
AC_C_INLINE
AM_PROG_CC_C_O
AC_DEFINE([_GNU_SOURCE], [], [Use GNU extensions])

# Library versioning
lt_major="0"
lt_revision="0"
lt_age="0"
AC_SUBST(lt_major)
AC_SUBST(lt_revision)
AC_SUBST(lt_age)

all_drivers="upekts upektc upeksonly vcom5s validity uru4000 fdu2000 aes1610 aes2501 aes4000"

require_imagemagick='no'
require_aeslib='no'
enable_upekts='no'
enable_upektc='no'
enable_upeksonly='no'
enable_vcom5s='no'
enable_validity='no'
enable_uru4000='no'
enable_fdu2000='no'
enable_aes1610='no'
enable_aes2501='no'
enable_aes4000='no'

AC_ARG_WITH([drivers],[AS_HELP_STRING([--with-drivers],
	[List of drivers to enable])],
	[drivers="$withval"],
	[drivers="$all_drivers"])

for driver in `echo ${drivers} | sed -e 's/,/ /g' -e 's/,$//g'`; do
	case ${driver} in
		upekts)
			AC_DEFINE([ENABLE_UPEKTS], [], [Build UPEK TouchStrip driver])
			enable_upekts="yes"
		;;
		upektc)
			AC_DEFINE([ENABLE_UPEKTC], [], [Build UPEK TouchChip driver])
			enable_upektc="yes"
		;;
		upeksonly)
			AC_DEFINE([ENABLE_UPEKSONLY], [], [Build UPEK TouchStrip sensor-only driver])
			enable_upeksonly="yes"
		;;
		uru4000)
			AC_DEFINE([ENABLE_URU4000], [], [Build Digital Persona U.are.U 4000 driver])
			enable_uru4000="yes"
		;;
		fdu2000)
			AC_DEFINE([ENABLE_FDU2000], [], [Build Secugen FDU 2000 driver])
			enable_fdu2000="yes"
		;;
		vcom5s)
			AC_DEFINE([ENABLE_VCOM5S], [], [Build Veridicom 5thSense driver])
			enable_vcom5s="yes"
		;;
		validity)
			AC_DEFINE([ENABLE_VALIDITY], [], [Build Validity driver])
			enable_validity="yes"
		;;
		aes2501)
			AC_DEFINE([ENABLE_AES2501], [], [Build AuthenTec AES2501 driver])
			require_aeslib="yes"
			enable_aes2501="yes"
		;;
		aes1610)
			AC_DEFINE([ENABLE_AES1610], [], [Build AuthenTec AES1610 driver])
			require_aeslib="yes"
			enable_aes1610="yes"
		;;
		aes4000)
			AC_DEFINE([ENABLE_AES4000], [], [Build AuthenTec AES4000 driver])
			require_aeslib="yes"
			require_imagemagick="yes"
			enable_aes4000="yes"
		;;
	esac
done

AM_CONDITIONAL([ENABLE_UPEKTS], [test "$enable_upekts" != "no"])
#AM_CONDITIONAL([ENABLE_UPEKTC], [test "$enable_upektc" != "no"])
AM_CONDITIONAL([ENABLE_UPEKSONLY], [test "$enable_upeksonly" != "no"])
AM_CONDITIONAL([ENABLE_VCOM5S], [test "$enable_vcom5s" != "no"])
AM_CONDITIONAL([ENABLE_VALIDITY], [test "$enable_validity" != "no"])
AM_CONDITIONAL([ENABLE_URU4000], [test "$enable_uru4000" != "no"])
#AM_CONDITIONAL([ENABLE_FDU2000], [test "$enable_fdu2000" != "no"])
#AM_CONDITIONAL([ENABLE_AES1610], [test "$enable_aes1610" != "no"])
AM_CONDITIONAL([ENABLE_AES2501], [test "$enable_aes2501" != "no"])
AM_CONDITIONAL([ENABLE_AES4000], [test "$enable_aes4000" != "no"])
AM_CONDITIONAL([REQUIRE_IMAGEMAGICK], [test "$require_imagemagick" != "no"])
AM_CONDITIONAL([REQUIRE_AESLIB], [test "$require_aeslib" != "no"])


PKG_CHECK_MODULES(LIBUSB, [libusb-1.0 >= 0.9.1])
AC_SUBST(LIBUSB_CFLAGS)
AC_SUBST(LIBUSB_LIBS)

# check for OpenSSL's libcrypto
PKG_CHECK_MODULES(CRYPTO, "libcrypto")
AC_SUBST(CRYPTO_CFLAGS)
AC_SUBST(CRYPTO_LIBS)

PKG_CHECK_MODULES(GLIB, "glib-2.0")
AC_SUBST(GLIB_CFLAGS)
AC_SUBST(GLIB_LIBS)

if test "$require_imagemagick" != "no"; then
PKG_CHECK_MODULES(IMAGEMAGICK, "ImageMagick")
AC_SUBST(IMAGEMAGICK_CFLAGS)
AC_SUBST(IMAGEMAGICK_LIBS)
fi;

# Examples build
AC_ARG_ENABLE([examples-build], [AS_HELP_STRING([--enable-examples-build],
	[build example applications (default n)])],
	[build_examples=$enableval],
	[build_examples='no'])
AM_CONDITIONAL([BUILD_EXAMPLES], [test "x$build_examples" != "xno"])

# Examples build
AC_ARG_ENABLE([x11-examples-build], [AS_HELP_STRING([--enable-x11-examples-build],
	[build X11 example applications (default n)])],
	[build_x11_examples=$enableval],
	[build_x11_examples='no'])
AM_CONDITIONAL([BUILD_X11_EXAMPLES], [test "x$build_x11_examples" != "xno"])


if test "x$build_x11_examples" != "xno"; then
	# check for Xv extensions
	# imported from Coriander
	AC_DEFUN([AC_CHECK_XV],[
		AC_SUBST(XV_CFLAGS)
		AC_SUBST(XV_LIBS)
		AC_MSG_CHECKING(for Xv extensions)
		AC_TRY_COMPILE([
		#include <X11/Xlib.h>
	 	#include <X11/extensions/Xvlib.h>],[
		int main(void) { (void) XvGetPortAttribute(0, 0, 0, 0); return 0; }
		],xv=yes,xv=no);
		AC_MSG_RESULT($xv)
	 	if test x$xv = xyes; then
    	       XV_LIBS="-lXv -lXext"
        	   XV_CFLAGS=""
		   AC_DEFINE(HAVE_XV,1,[defined if XV video overlay is available])
		else
			AC_MSG_ERROR([XV is required for X11 examples])
 		fi
	])
	AC_CHECK_XV
fi

# Message logging
AC_ARG_ENABLE([log], [AS_HELP_STRING([--disable-log], [disable all logging])],
	[log_enabled=$enableval],
	[log_enabled='yes'])
if test "x$log_enabled" != "xno"; then
	AC_DEFINE([ENABLE_LOGGING], 1, [Message logging])
fi

AC_ARG_ENABLE([debug-log], [AS_HELP_STRING([--enable-debug-log],
	[enable debug logging (default n)])],
	[debug_log_enabled=$enableval],
	[debug_log_enabled='no'])
if test "x$debug_log_enabled" != "xno"; then
	AC_DEFINE([ENABLE_DEBUG_LOGGING], 1, [Debug message logging])
fi

# Restore gnu89 inline semantics on gcc 4.3 and newer
saved_cflags="$CFLAGS"
CFLAGS="$CFLAGS -fgnu89-inline"
AC_COMPILE_IFELSE(AC_LANG_PROGRAM([]), inline_cflags="-fgnu89-inline", inline_cflags="")
CFLAGS="$saved_cflags"

AC_DEFINE([API_EXPORTED], [__attribute__((visibility("default")))], [Default visibility])
AM_CFLAGS="-std=gnu99 $inline_cflags -Wall -Wundef -Wunused -Wstrict-prototypes -Werror-implicit-function-declaration -Wno-pointer-sign -Wshadow"
AC_SUBST(AM_CFLAGS)

AC_CONFIG_FILES([libfprint.pc] [Makefile] [libfprint/Makefile] [examples/Makefile] [doc/Makefile])
AC_OUTPUT


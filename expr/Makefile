ALL:
	cl  /GS /analyze- /W3 /Zc:wchar_t /I"C:\Users\ARC\Downloads\NatNet_SDK_3.1\NatNetSDK\include" /Zi /Gm- /O2 \
		/Zc:inline /fp:precise /D "NATNETLIB_IMPORTS" /D "WIN32" /D "_CRT_NONSTDC_NO_DEPRECATE" \
			/D "_CRT_SECURE_NO_WARNINGS" /D "NDEBUG" /D "_VC80_UPGRADE=0x0710" /errorReport:prompt \
				/WX- /Zc:forScope /Gd /Oy- /MD /FC  /EHsc /nologo  \
				expr.cpp \
					/link /DYNAMICBASE "NatNetLib.lib" "ws2_32.lib" "kernel32.lib" "user32.lib" "gdi32.lib" \
					"winspool.lib" "comdlg32.lib" "advapi32.lib" "shell32.lib" "ole32.lib" "oleaut32.lib" \
					"uuid.lib" "odbc32.lib" "odbccp32.lib" \
					/LIBPATH:"C:\Users\ARC\Downloads\NatNet_SDK_3.1\NatNetSDK\lib\x86" /TLBID:1 

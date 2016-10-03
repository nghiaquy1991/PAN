# invoke SourceDir generated makefile for app.pem3
app.pem3: .libraries,app.pem3
.libraries,app.pem3: package/cfg/app_pem3.xdl
	$(MAKE) -f C:\ti\simplelink\ti-15.4-stack-sdk_2_00_00_25\examples\sensor\cc13xx\ccs-rfd/src/makefile.libs

clean::
	$(MAKE) -f C:\ti\simplelink\ti-15.4-stack-sdk_2_00_00_25\examples\sensor\cc13xx\ccs-rfd/src/makefile.libs clean


# invoke SourceDir generated makefile for app.pem3
app.pem3: .libraries,app.pem3
.libraries,app.pem3: package/cfg/app_pem3.xdl
	$(MAKE) -f D:\Git\PAN\sensor_cc13xx_lp/src/makefile.libs

clean::
	$(MAKE) -f D:\Git\PAN\sensor_cc13xx_lp/src/makefile.libs clean


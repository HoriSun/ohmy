ecProject(mkControlPluginGui)

ecIncludeSolutions(
    stableFoundation
    actinCore
    foundation
    actin
    sensor
    render
    guiCommon
    plugins
    mk
)

ecExternIncludeDirs(Boost osg QT RTOS32Win)

ecProjectLinkLibraries(
    guiCommon-viewerCore
    sensor-signalFilter
    mk-mkControlPluginExec
    plugins-manipulationDirectorPlugin
)

ecExternLinkLibraries(
    Qt5Widgets
)

ecSourceFiles(
    mkControlPluginGui.cpp
    ecMkControlWidget.cpp
)

ecQtUiFiles(
    ecMkControlWidget.ui
)

ecQtObjectHeaderFiles(
    mkControlPluginGui.h
    ecMkControlWidget.h
)

ecPlugin(0)

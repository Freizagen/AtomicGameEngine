var fs = require('fs');
var path = require('path');

var jakeRoot = __dirname;

var artifactsFolder = jakeRoot + "/Artifacts";

// cmake
var cmakeDevBuild = false;

// build folder
var windowsBuildFolder = artifactsFolder + "/Windows_Build";
var macOSXBuildFolder = artifactsFolder + "/MacOSX_Build";
var androidBuildFolder = artifactsFolder + "/Android_Build";
var iosBuildFolder = artifactsFolder + "/IOS_Build";
var webBuildFolder = artifactsFolder + "/Web_Build";
var linuxBuildFolder = artifactsFolder + "/Linux_Build";

var atomictool = macOSXBuildFolder + "/Source/AtomicTool/Release/AtomicTool"

var allBuildFolders = [
  windowsBuildFolder,
  macOSXBuildFolder,
  androidBuildFolder,
  iosBuildFolder,
  webBuildFolder,
  linuxBuildFolder
];

// packaging
var windowsPackageFolder = artifactsFolder + "/Windows_Package";
var macOSXPackageFolder = artifactsFolder + "/MacOSX_Package";

var allPackageFolders = [
  windowsPackageFolder,
  macOSXPackageFolder
];

function getAtomicDevBuildDefine() {

  return "-DATOMIC_DEV_BUILD=" + (cmakeDevBuild ? "1" : "0");

}

function testRemoveDir(path) {

  if (fs.existsSync(path)) {
    jake.rmRf(path);
  }

  if (fs.existsSync(path)) {

    throw "Unable to remove path: " + path;

  }
}

namespace('clean', function() {

  task('all', {async:true}, function() {

    for (var i in allBuildFolders) {
      testRemoveDir(allBuildFolders[i]);
    }

    for (var i in allPackageFolders) {
      testRemoveDir(allPackageFolders[i]);
    }

    complete();

  });
});

namespace('build', function() {

  task('macosx_atomictool', {async:true}, function() {

    if (!fs.existsSync(macOSXBuildFolder)) {
      jake.mkdirP(macOSXBuildFolder);
    }

    var cmds = [
      'cmake ../../ -G Xcode ' + getAtomicDevBuildDefine(),
      'xcodebuild -target AtomicTool -configuration Release'
    ]

    process.chdir(macOSXBuildFolder);

    jake.exec(cmds, function() {
      console.log("Built MacOSX");
      complete();
    }, {
      printStdout: true
    });

  }); // end build:macosx_atomictool


  task('macosx', {async:true}, function() {

    if (!fs.existsSync(macOSXBuildFolder)) {
      jake.mkdirP(macOSXBuildFolder);
    }

    var cmds = [
      'cmake ../../ -G Xcode ' + getAtomicDevBuildDefine(),
      'xcodebuild -configuration Release'
    ]

    process.chdir(macOSXBuildFolder);

    jake.exec(cmds, function() {
      console.log("Built MacOSX");
      complete();
    }, {
      printStdout: true
    });

  }); // end build:macosx

  task('windows', {async:true}, function() {

    if (!fs.existsSync(windowsBuildFolder)) {
      jake.mkdirP(windowsBuildFolder);
    }

    process.chdir(windowsBuildFolder);

    jake.exec(jakeRoot + "/Build/Windows/Compile.bat", function() {
      complete();
    }, {
      printStdout: true
    });

  });

  task('android', ['macosx_atomictool'], {async:true}, function() {

    if (!fs.existsSync(androidBuildFolder)) {
      jake.mkdirP(androidBuildFolder);
    }

    process.chdir(androidBuildFolder);

    var cmds = [
      atomictool + " bind " + jakeRoot + " Script/Packages/Atomic/ ANDROID",
      atomictool + " bind " + jakeRoot + " Script/Packages/AtomicPlayer/ ANDROID",
      "cmake -DCMAKE_TOOLCHAIN_FILE=" + jakeRoot + "/CMake/Toolchains/android.toolchain.cmake -DCMAKE_BUILD_TYPE=Release ../../",
      "make -j4"
    ]

    jake.exec(cmds, function() {
      console.log("Built Android");
      complete();
    }, {
      printStdout: true
    });

  });

/*
  task :android =>  "build:macosx_atomictool" do

      if !Dir.exists?("#{CMAKE_ANDROID_BUILD_FOLDER}")
        FileUtils.mkdir_p(CMAKE_ANDROID_BUILD_FOLDER)
      end

      Dir.chdir(CMAKE_ANDROID_BUILD_FOLDER) do

        sh "#{ATOMICTOOL_BIN_MACOSX} bind #{$RAKE_ROOT} Script/Packages/Atomic/ ANDROID"
        sh "#{ATOMICTOOL_BIN_MACOSX} bind #{$RAKE_ROOT} Script/Packages/AtomicPlayer/ ANDROID"
        sh "cmake -DCMAKE_TOOLCHAIN_FILE=#{$RAKE_ROOT}/CMake/Toolchains/android.toolchain.cmake -DCMAKE_BUILD_TYPE=Release ../../"
        sh "make -j4"
      end

  end
*/


}); // end build namespace


namespace('package', function() {

  task('macosx', ['clean:all', 'build:macosx'], function() {

    jake.mkdirP(macOSXPackageFolder + "/AtomicEditor.app/Contents/Resources/");

    var editorAppSourceFolder = macOSXBuildFolder + "/Source/AtomicEditor/Release/AtomicEditor.app";

    // copy app (using cp here as jake.cpR seems to strip executable flag)
    jake.exec("cp -r " + editorAppSourceFolder + " " + macOSXPackageFolder, {
      printStdout: true
    });

    // copy resources

    jake.cpR(jakeRoot + "/Resources/CoreData", macOSXPackageFolder + "/AtomicEditor.app/Contents/Resources/");
    jake.cpR(jakeRoot + "/Resources/EditorData", macOSXPackageFolder + "/AtomicEditor.app/Contents/Resources/");
    jake.cpR(jakeRoot + "/Resources/PlayerData", macOSXPackageFolder + "/AtomicEditor.app/Contents/Resources/");
    jake.cpR(jakeRoot + "/Data/AtomicEditor/", macOSXPackageFolder + "/AtomicEditor.app/Contents/Resources/ToolData/");

  });

  task('windows', ['clean:all', 'build:windows'], function() {


    jake.mkdirP(windowsPackageFolder + "/Resources");

    jake.cpR(windowsBuildFolder + "/Source/AtomicEditor/Release/AtomicEditor.exe", windowsPackageFolder + "/AtomicEditor.exe" )

    // 32 bit build for packaging!

    jake.cpR(jakeRoot + "/Build/Windows/Binaries/x86/D3DCompiler_47.dll", windowsPackageFolder + "/D3DCompiler_47.dll" )

    // copy resources

    jake.cpR(jakeRoot + "/Resources/CoreData",  windowsPackageFolder + "/Resources");
    jake.cpR(jakeRoot + "/Resources/EditorData", windowsPackageFolder + "/Resources");
    jake.cpR(jakeRoot + "/Resources/PlayerData", windowsPackageFolder + "/Resources");
    jake.cpR(jakeRoot + "/Script", windowsPackageFolder + "/Resources");
    jake.cpR(jakeRoot + "/Data/AtomicEditor/", windowsPackageFolder + "/Resources/ToolData/");
    jake.cpR(jakeRoot + "/Data/AtomicEditor/ProjectTemplates", windowsPackageFolder + "/Resources");


  });


});

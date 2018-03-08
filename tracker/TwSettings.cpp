#include "TwSettings.h"
#include "util/gl_wrapper.h"
#include "util/mylogger.h"
#ifdef WITH_ANTTWEAKBAR
    #include "AntTweakBar.h"
#endif

///@{ Definition of extern settings
TwSettings _settings;
TwSettings* tw_settings = &_settings;
///@}

void TwSettings::tw_init(int width, int height){
#ifdef WITH_ANTTWEAKBAR
    // LOG(INFO) << "TwSettings::tw_init";
    TwInit(TW_OPENGL_CORE, NULL);
    TwWindowSize(width, height);
    this->bar = TwNewBar("Settings");
#endif
}

void TwSettings::tw_cleanup(){
#ifdef WITH_ANTTWEAKBAR
    TwTerminate();
#endif
}

void TwSettings::tw_resz(int width, int height){
#ifdef WITH_ANTTWEAKBAR
     TwWindowSize(width, height);
#endif
}

void TwSettings::tw_draw(){
#ifdef WITH_ANTTWEAKBAR
    glDisable(GL_DEPTH_TEST);
        TwDraw();
    glEnable(GL_DEPTH_TEST);
#endif
}

void TwSettings::tw_add(float& var, const char* name, const char* def){
    if(bar==NULL) return;
#ifdef WITH_ANTTWEAKBAR
    TwAddVarRW(bar, name, TW_TYPE_FLOAT, &var, def);
#endif
}

void TwSettings::tw_add(bool& var, const char* name, const char* def){
	//std::cout << "tw_add: " << name << std::endl;
    if(bar==NULL) return;
	if (name[5] == 'w')
		std::cout << " bar var: " << name << " address: " << &var << std::endl;
#ifdef WITH_ANTTWEAKBAR
    int err =  TwAddVarRW(bar, name, TW_TYPE_BOOLCPP, &var, def);
	
#endif
}

void TwSettings::tw_add(int& var, const char* name, const char* def){
    if(bar==NULL) return;
#ifdef WITH_ANTTWEAKBAR
    TwAddVarRW(bar, name, TW_TYPE_INT32, &var, def);
#endif
}

void TwSettings::tw_add_ro(float& var, const char* name, const char* def){
    if(bar==NULL) return;
#ifdef WITH_ANTTWEAKBAR
    TwAddVarRO(bar, name, TW_TYPE_FLOAT, &var, def);
#endif
}

void TwSettings::tw_add_ro(bool& var, const char* name, const char* def){
    if(bar==NULL) return;
#ifdef WITH_ANTTWEAKBAR
    TwAddVarRO(bar, name, TW_TYPE_BOOLCPP, &var, def);
#endif
}

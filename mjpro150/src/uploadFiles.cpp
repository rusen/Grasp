// Application that uploads anything in current data folder to the web server.
#include <util/util.h>


int main(int argc, const char** argv){

	while(true)
	{
	    // There may be extra files that need to be uploaded
	    // (by other clients which do not have direct internet access).
	    // Upload them too.
	    Grasp::UploadExtraFiles(argv[1]);

	    // Sleep for 10 seconds and try again.
		std::this_thread::sleep_for(std::chrono::seconds(10));
	}

}

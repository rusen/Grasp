#include <util/util.h>
#include <util/Connector.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <stdio.h>
#include <random>
#include <boost/filesystem.hpp>

#define PI 3.141592
#define RF (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))

namespace Grasp{

void replaceAll(std::string& str, const std::string& from, const std::string& to) {
    if(from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length(); // In case 'to' contains 'from', like replacing 'x' with 'yx'
    }
}

// Function to create relevant XMLs with random parameters.
std::string CreateXMLs(const char * base, GraspPlanner * planner, int objectId, int baseId){
	// Templates for model, base and assets files are already there.
	// We just need to modify them by adding the variations.

	// Allocate space for model xml.
	char modelXMLStr[10000];
	modelXMLStr[0] = 0;
	sprintf(modelXMLStr, "<mujoco model=\"BHAM\">\n    <include file=\"include_BHAM.xml\"/>\n    <include file=\"include_assets.xml\"/>\n");

    // Manage file names
    char baseIdFile[1000], classIdFile[1000], oldAssetFile[1000], oldBaseAssetFile[1000], newAssetFile[1000],
	newBaseAssetFile[1000], oldObjectFile[1000], oldBaseFile[1000], newObjectFile[1000], newBaseFile[1000],
	tmp[1000], tmpXML[1000], lightFile[1000], tableFile[1000], modelPrefix[1000];

    // Get base id file name
    strcpy(baseIdFile, base);
    strcat(baseIdFile, "/baseIds.txt");

    // Read base id
    int baseType = 0;
    FILE * fid = fopen(baseIdFile, "r");
    for (int i = 0; i<objectId; i++)
    	fscanf(fid, "%d\n", &baseType);
    fclose(fid);

    // Get base id file name
    strcpy(classIdFile, base);
    strcat(classIdFile, "/classIds.txt");

    // Read class id
    int classId = 0;
    fid = fopen(classIdFile, "r");
    for (int i = 0; i<objectId; i++)
    	fscanf(fid, "%d\n", &classId);
    classId = classId - 1;
    fclose(fid);

    // Obtain a filename prefix for this specific object
    strcpy(modelPrefix, base);
    strcat(modelPrefix, "/");
    strcat(modelPrefix, planner->fileId);
    strcat(modelPrefix, "_");

    // Get light and table for this specific object
    strcpy(tmp, base);
    strcat(tmp, "/include_lightOrg.xml");
    strcpy(lightFile, modelPrefix);
    strcat(lightFile, "include_light.xml");
    sprintf(tmpXML, "    <include file=\"%s_include_light.xml\"/>\n", planner->fileId);
    strcat(modelXMLStr, tmpXML);
    boost::filesystem::copy_file(tmp, lightFile, boost::filesystem::copy_option::overwrite_if_exists);
    strcpy(tmp, base);
    strcat(tmp, "/include_tableOrg.xml");
    strcpy(tableFile, modelPrefix);
    strcat(tableFile, "include_table.xml");
    sprintf(tmpXML, "    <include file=\"%s_include_table.xml\"/>\n", planner->fileId);
    strcat(modelXMLStr, tmpXML);
    boost::filesystem::copy_file(tmp, tableFile, boost::filesystem::copy_option::overwrite_if_exists);

    // Object model
    strcpy(newObjectFile, modelPrefix);
    strcat(newObjectFile, "include_object.xml");
    strcpy(oldObjectFile, base);
    strcat(oldObjectFile, "/mesh/objects/");
    sprintf(tmp, "D_%d/D_%d_object.xml", objectId, objectId);
    strcat(oldObjectFile, tmp);
    sprintf(tmpXML, "    <include file=\"%s_include_object.xml\"/>\n", planner->fileId);
    strcat(modelXMLStr, tmpXML);
    boost::filesystem::copy_file(oldObjectFile, newObjectFile, boost::filesystem::copy_option::overwrite_if_exists);

    // Object assets
    strcpy(newAssetFile, modelPrefix);
    strcat(newAssetFile, "include_object_assets.xml");
    strcpy(oldAssetFile, base);
    strcat(oldAssetFile, "/mesh/objects/");
    sprintf(tmp, "D_%d/D_%d_assets.xml", objectId, objectId);
    strcat(oldAssetFile, tmp);
    sprintf(tmpXML, "    <include file=\"%s_include_object_assets.xml\"/>\n", planner->fileId);
    strcat(modelXMLStr, tmpXML);
    boost::filesystem::copy_file(oldAssetFile, newAssetFile, boost::filesystem::copy_option::overwrite_if_exists);

    // Depending on the base type, get relevant base files.
    if (baseType == 1)
    {
    	// Set minimum z to just above the base platform (usually a cup)
    	planner->minPointZ = -0.255; // Table is at -0.35, each base is about 9 cms.

    	// Base file
        strcpy(newBaseFile, modelPrefix);
        strcat(newBaseFile, "include_base.xml");
        strcpy(oldBaseFile, base);
        strcat(oldBaseFile, "/mesh/bases/");
        sprintf(tmp, "D_%d/D_%d_base.xml", baseId, baseId);
        strcat(oldBaseFile, tmp);
        sprintf(tmpXML, "    <include file=\"%s_include_base.xml\"/>\n", planner->fileId);
        strcat(modelXMLStr, tmpXML);
        boost::filesystem::copy_file(oldBaseFile, newBaseFile, boost::filesystem::copy_option::overwrite_if_exists);

		// Base assets
		strcpy(newBaseAssetFile, modelPrefix);
		strcat(newBaseAssetFile, "include_base_assets.xml");
		strcpy(oldBaseAssetFile, base);
		strcat(oldBaseAssetFile, "/mesh/bases/");
		sprintf(tmp, "D_%d/D_%d_assets.xml", baseId, baseId);
		strcat(oldBaseAssetFile, tmp);
        sprintf(tmpXML, "    <include file=\"%s_include_base_assets.xml\"/>\n", planner->fileId);
        strcat(modelXMLStr, tmpXML);
		boost::filesystem::copy_file(oldBaseAssetFile, newBaseAssetFile, boost::filesystem::copy_option::overwrite_if_exists);
    }
    else if (baseType == 2)
    {
    	strcat(modelXMLStr, "    <include file=\"include_baseInv.xml\"/>\n");
    }

    // Finish and print model xml.
    strcat(modelXMLStr, "</mujoco>");
    strcpy(tmp, modelPrefix);
    strcat(tmp, "Test.xml");
	std::ofstream tOut(tmp);
	tOut<<modelXMLStr;
	tOut.close();

	// Create output string.
	std::string outputStr(tmp);

    // Modify model file.
	std::ifstream t(newObjectFile);
	std::string objectStr((std::istreambuf_iterator<char>(t)),
					 std::istreambuf_iterator<char>());

	// Create random friction
	float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	r = r / 2 + 0.5;  // random number between 0.5 and 1;
	sprintf(tmp, "friction=\"%f 0.005 0.0001\"", r);

	// Replace friction
	replaceAll(objectStr, std::string("friction=\"\""), std::string(tmp));

    // Scale the object randomly.
	std::ifstream tAsset(newAssetFile);
	std::string assetStr((std::istreambuf_iterator<char>(tAsset)),
					 std::istreambuf_iterator<char>());
	tAsset.close();

	// Create random scale based on the object type.
	float xScale = RF/2+0.75;
	float yScale = xScale;
	float zScale = RF/2+0.75;
	float eulerx = 0, eulery = 0, eulerz = RF * 2 * M_PI;
	switch(classId)
	{
	case 0: // Bottle
		zScale = RF/2+0.8;
		yScale = RF*0.2+0.9;
		xScale = yScale;
		break;
	case 1: // Bowl
		xScale = RF+0.5; // make it all proportional
		yScale = xScale;
		zScale = xScale;
		break;
	case 2: // Cup
		zScale = RF/2+1; // longer
		yScale = RF*0.5+0.6;
		xScale = yScale;
		break;
	case 3: // Fork
		break; // default
	case 4: // Pan
		xScale = RF/2+0.5;
		yScale = xScale;
		zScale = RF/2+1;
		break;
	case 5: // Jug
		xScale = RF*0.3+0.7; // slightly smaller
		yScale = xScale;
		zScale = xScale;
		break;
	case 6: // Knife
		break; // default
	case 7: // Mug
		xScale = RF*0.1+0.7; // make it all proportional
		yScale = xScale;
		zScale = xScale;
		eulerx = RF * M_PI;
		eulery = RF * M_PI;
		break;
	case 8: //Plate
		xScale = RF*0.5+0.75; // make it all proportional
		yScale = xScale;
		zScale = xScale;
		break;
	case 9: // Scissors
		xScale = RF*0.5+0.75; // make it all proportional
		yScale = xScale;
		zScale = xScale;
		break; //
	case 10: // Shaker
		xScale = RF*0.2+0.9; // make it all proportional
		yScale = xScale;
		zScale = xScale;
		break;
	case 11: //Spatula
		break; // default
	case 12: // Spoon
		break; // default
	case 13: // Teacup
		xScale = RF*0.4+0.9; // make it all proportional, slightly bigger
		yScale = xScale;
		zScale = xScale;
		break;
	case 14: // Teapot
		xScale = RF*0.3+0.7; // slightly smaller
		yScale = xScale;
		zScale = xScale;
		break;
	}

	// Replace scale
	sprintf(tmp, "<mesh scale=\"%f %f %f\" ", xScale, yScale, zScale);
	replaceAll(assetStr, std::string("<mesh "), std::string(tmp));

	// Write light string.
	std::ofstream tAssetOut(newAssetFile);
	tAssetOut<<assetStr;
	tAssetOut.close();

	// Create random mass, depending on the object type.
	float baseWeights[] = {30, 50, 30, 40, 200, 80, 50, 650, 40, 50, 100, 40, 40, 120, 800};
	float addedWeights[] = {40, 350, 300, 40, 330, 90, 100, 100, 80, 100, 60, 40, 40, 80, 400};

	int baseWeight = baseWeights[classId];
	int addedWeight = addedWeights[classId];
	float mass = (float)(rand()%addedWeight + baseWeight);
	mass = mass/1000;
	mass = mass * (xScale * yScale * zScale); // Scale with object size
	sprintf(tmp, "mass=\"%f\"", mass);

	// Replace mass
	replaceAll(objectStr, std::string("mass=\"\""), std::string(tmp));

	// Create random orientation
	sprintf(tmp, "euler=\"%f %f %f\"", eulerx, eulery, eulerz);
	replaceAll(objectStr, std::string("euler=\"\""), std::string(tmp));

	// Create random object colour
	float red = ((float)(rand()%255))/255.0, green = ((float)(rand()%255))/255.0, blue = ((float)(rand()%255))/255.0;
	sprintf(tmp, "rgba=\"%f %f %f 1\"", red, green, blue);
	replaceAll(objectStr, std::string("rgba=\"\""), std::string(tmp));

	// Write object file.
	t.close();
	std::ofstream tOutObject(newObjectFile);
	tOutObject<<objectStr;
	tOutObject.close();

	// Moving on to the light.
	std::ifstream tLight(lightFile);
	std::string lightStr((std::istreambuf_iterator<char>(tLight)),
	                 std::istreambuf_iterator<char>());
	tLight.close();
	// Assign intensity [ 0.5 - 1]
	float intensity = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	intensity = intensity/2 + 0.5;
	sprintf(tmp, "diffuse=\"%f %f %f\"", intensity, intensity, intensity);
	replaceAll(lightStr, std::string("diffuse=\"\""), std::string(tmp));

	// Specular intensity of the light
	intensity = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	intensity = intensity/4;
	sprintf(tmp, "specular=\"%f %f %f\"", intensity, intensity, intensity);
	replaceAll(lightStr, std::string("specular=\"\""), std::string(tmp));

	// Assign location (-0.5:0.5, -0.5:0.5, 1:2)
	float x = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	x = x - 0.5;
	float y = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	y = y - 0.5;
	float z = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	z = z + 0.9;
	sprintf(tmp, "pos=\"%f %f %f\"", x, y, z);
	replaceAll(lightStr, std::string("pos=\"\""), std::string(tmp));

	// Assign lookat point for the light.
	x = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	x = (x) - 0.5;
	y = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	y = (y) - 0.5;
	z = -1;
	sprintf(tmp, "dir=\"%f %f %f\"", x, y, z);
	replaceAll(lightStr, std::string("dir=\"\""), std::string(tmp));

	// Write light string.
	std::ofstream tLightOut(lightFile);
	tLightOut<<lightStr;
	tLightOut.close();

	// Fix the table colour as well.
	std::ifstream tTable(tableFile);
	std::string tableStr((std::istreambuf_iterator<char>(tTable)),
	                 std::istreambuf_iterator<char>());
	tTable.close();
	intensity = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	intensity = intensity/2 + 0.3; // [0.3 - 0.8]
	sprintf(tmp, "rgba=\"%f %f %f 1\"", intensity, intensity, intensity);
	replaceAll(tableStr, std::string("rgba=\"\""), std::string(tmp));

	// Write table string.
	std::ofstream tTableOut(tableFile);
	tTableOut<<tableStr;
	tTableOut.close();

	if (baseType == 1)
	{
		// Finally, we fix the base colour.
		std::ifstream tBase(newBaseFile);
		std::string baseStr((std::istreambuf_iterator<char>(tBase)),
						 std::istreambuf_iterator<char>());
		tBase.close();
		red = ((float)(rand()%255))/255.0, green = ((float)(rand()%255))/255.0, blue = ((float)(rand()%255))/255.0;
		sprintf(tmp, "rgba=\"%f %f %f 1\"", red, green, blue);
		replaceAll(baseStr, std::string("rgba=\"\""), std::string(tmp));

		// Write table string.
		std::ofstream tBaseOut(newBaseFile);
		tBaseOut<<baseStr;
		tBaseOut.close();
	}

	// Return
	return outputStr;
}

void UploadFiles(const char * base, GraspPlanner * planner, int objectId, int baseId){
    // Before we move on to grasping loop, we save all the model files to the cloud.
    char tmpStr[1000], tmpStr2[1000], modelPrefix[1000], dataPrefix[1000], dataPrefix2[1000], selfDropboxFolder[1000];
    bool uploadSuccess = true;

    // Obtain a filename prefix for this specific object
    strcpy(modelPrefix, base);
    strcat(modelPrefix, "/");
    strcat(modelPrefix, planner->fileId);
    strcat(modelPrefix, "_");

    // Create temp prefix
    strcpy(dataPrefix, planner->dropboxFolder);
    strcat(dataPrefix, "/upload/");
    strcat(dataPrefix, planner->fileId);
    strcat(dataPrefix, "_");
    strcpy(dataPrefix2, planner->dropboxFolder);
    strcat(dataPrefix2, "/upload/");
    strcat(dataPrefix2, planner->fileId);

    // Upload all files
    // Test xml file
    strcpy(tmpStr, modelPrefix);
    strcpy(tmpStr2, dataPrefix);
    strcat(tmpStr, "Test.xml");
    strcat(tmpStr2, "Test.xml");
    bool fileSuccess = Connector::UploadFile(tmpStr);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(tmpStr))
    {
        std::cout<<"Copying "<<tmpStr<<" to "<<tmpStr2<<"."<<std::endl;
        boost::filesystem::copy_file(tmpStr, tmpStr2);
    	boost::filesystem::remove(tmpStr);
    }

    // Light
    strcpy(tmpStr, modelPrefix);
    strcpy(tmpStr2, dataPrefix);
    strcat(tmpStr, "include_light.xml");
    strcat(tmpStr2, "include_light.xml");
    fileSuccess = Connector::UploadFile(tmpStr);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(tmpStr))
    {
        std::cout<<"Copying "<<tmpStr<<" to "<<tmpStr2<<"."<<std::endl;
        boost::filesystem::copy_file(tmpStr, tmpStr2);
        boost::filesystem::remove(tmpStr);
    }

    // Table
    strcpy(tmpStr, modelPrefix);
    strcpy(tmpStr2, dataPrefix);
    strcat(tmpStr, "include_table.xml");
    strcat(tmpStr2, "include_table.xml");
    fileSuccess = Connector::UploadFile(tmpStr);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(tmpStr))
    {
        std::cout<<"Copying "<<tmpStr<<" to "<<tmpStr2<<"."<<std::endl;
        boost::filesystem::copy_file(tmpStr, tmpStr2);
        boost::filesystem::remove(tmpStr);
    }

    // Object
    strcpy(tmpStr, modelPrefix);
    strcpy(tmpStr2, dataPrefix);
    strcat(tmpStr, "include_object.xml");
    strcat(tmpStr2, "include_object.xml");
    fileSuccess = Connector::UploadFile(tmpStr);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(tmpStr))
    {
        std::cout<<"Copying "<<tmpStr<<" to "<<tmpStr2<<"."<<std::endl;
        boost::filesystem::copy_file(tmpStr, tmpStr2);
        boost::filesystem::remove(tmpStr);
    }

    // Object assets
    strcpy(tmpStr, modelPrefix);
    strcpy(tmpStr2, dataPrefix);
    strcat(tmpStr, "include_object_assets.xml");
    strcat(tmpStr2, "include_object_assets.xml");
    fileSuccess = Connector::UploadFile(tmpStr);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(tmpStr))
    {
        std::cout<<"Copying "<<tmpStr<<" to "<<tmpStr2<<"."<<std::endl;
        boost::filesystem::copy_file(tmpStr, tmpStr2);
        boost::filesystem::remove(tmpStr);
    }

    // Base
    strcpy(tmpStr, modelPrefix);
    strcpy(tmpStr2, dataPrefix);
    strcat(tmpStr, "include_base.xml");
    strcat(tmpStr2, "include_base.xml");
    fileSuccess = Connector::UploadFile(tmpStr);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(tmpStr))
    {
        std::cout<<"Copying "<<tmpStr<<" to "<<tmpStr2<<"."<<std::endl;
        boost::filesystem::copy_file(tmpStr, tmpStr2);
        boost::filesystem::remove(tmpStr);
    }

    // Base assets
    strcpy(tmpStr, modelPrefix);
    strcpy(tmpStr2, dataPrefix);
    strcat(tmpStr, "include_base_assets.xml");
    strcat(tmpStr2, "include_base_assets.xml");
    fileSuccess = Connector::UploadFile(tmpStr);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(tmpStr))
    {
        boost::filesystem::copy_file(tmpStr, tmpStr2);
        boost::filesystem::remove(tmpStr);
    }

    /*
    // Log file
    strcpy(tmpStr2, dataPrefix2);
    strcat(tmpStr2, ".log");
    fileSuccess = Connector::UploadFile(planner->logFile);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(planner->logFile))
    {
        boost::filesystem::copy_file(planner->logFile, tmpStr2);
        boost::filesystem::remove(planner->logFile);
    }
    */

    // Debug log file
    strcpy(tmpStr2, dataPrefix2);
    strcat(tmpStr2, "_debug.log");
    fileSuccess = Connector::UploadFile(planner->debugLogFile);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(planner->debugLogFile))
    {
        boost::filesystem::copy_file(planner->debugLogFile, tmpStr2);
        boost::filesystem::remove(planner->debugLogFile);
    }

    // Point cloud
    strcpy(tmpStr2, dataPrefix2);
    strcat(tmpStr2, ".pcd");
    fileSuccess = Connector::UploadFile(planner->pointFile);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(planner->pointFile))
    {
        boost::filesystem::copy_file(planner->pointFile, tmpStr2);
        boost::filesystem::remove(planner->pointFile);
    }

    // Image
    strcpy(tmpStr2, dataPrefix2);
    strcat(tmpStr2, "_rgb.png");
    fileSuccess = Connector::UploadFile(planner->rgbFile);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(planner->rgbFile))
    {
        boost::filesystem::copy_file(planner->rgbFile, tmpStr2);
        boost::filesystem::remove(planner->rgbFile);
    }

    // Depth
    strcpy(tmpStr2, dataPrefix2);
    strcat(tmpStr2, "_depth.png");
    fileSuccess = Connector::UploadFile(planner->depthFile);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(planner->depthFile))
    {
        boost::filesystem::copy_file(planner->depthFile, tmpStr2);
        boost::filesystem::remove(planner->depthFile);
    }

    // Grasp data
    strcpy(tmpStr2, dataPrefix2);
    strcat(tmpStr2, ".gd");
    fileSuccess = Connector::UploadFile(planner->resultFile);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(planner->resultFile))
    {
        boost::filesystem::copy_file(planner->resultFile, tmpStr2);
        boost::filesystem::remove(planner->resultFile);
    }

    // Trajectory
    strcpy(tmpStr2, dataPrefix2);
    strcat(tmpStr2, ".trj");
    fileSuccess = Connector::UploadFile(planner->trajectoryFile);
    uploadSuccess = uploadSuccess && fileSuccess;
    if (!fileSuccess && boost::filesystem::exists(planner->trajectoryFile))
    {
        boost::filesystem::copy_file(planner->trajectoryFile, tmpStr2);
        boost::filesystem::remove(planner->trajectoryFile);
    }

    // delete tmp folder
    if(boost::filesystem::exists(planner->baseFolder))
    {
       boost::filesystem::remove_all(planner->baseFolder);
    }
}

void UploadExtraFiles(const char * dropboxBase){
	char tmpStr[1000];
	strcpy(tmpStr, dropboxBase);
	strcat(tmpStr, "/upload");

	// No such folder? Return.
	if (!boost::filesystem::is_directory(tmpStr))
		return;

	// Try to upload all files under the $dropboxBase$/upload folder to the system.
	boost::filesystem::path p(tmpStr);
	boost::filesystem::directory_iterator end_itr;
	bool uploadFlag = true;
	char tmp[1000];

	// Take the first file you see.
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		if (i->path().extension().string() == ".pcd" || i->path().extension().string() == ".xml" || i->path().extension().string() == ".png" ||
				i->path().extension().string() == ".trj" || i->path().extension().string() == ".gd" || i->path().extension().string() == ".log")
		{
			std::cout<<"Uploading "<<i->path().string().c_str()<<std::endl;
			uploadFlag = uploadFlag && Connector::UploadFile(i->path().string().c_str());
			if (uploadFlag)
			{
				boost::filesystem::remove(i->path().string().c_str());
			}
			else
				break;
		}
	}
}


void RemoveOldFolders(const char * dropboxBase){
	char tmpStr[1000];
	strcpy(tmpStr, dropboxBase);
	strcat(tmpStr, "/data");

	// No such folder? Return.
	if (!boost::filesystem::is_directory(tmpStr))
		return;

	// Try to upload all files under the $dropboxBase$/upload folder to the system.
	boost::filesystem::path p(tmpStr);
	boost::filesystem::directory_iterator end_itr;
	bool uploadFlag = true;
	char tmp[1000];
	time_t curTime = time(NULL);

	// Take the first file you see.
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		time_t fileTime = boost::filesystem::last_write_time(i->path());
		if (curTime - fileTime > 1800){
			boost::filesystem::remove_all(i->path());
		}
	}
}

void RemoveOldTmpFolders(const char * modelFolder){

	// Remove all files from the model folder
	boost::filesystem::path p(modelFolder);
	time_t curTime = time(NULL);

	// Go over the files and delete them one by one.
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		time_t fileTime = boost::filesystem::last_write_time(i->path());
		if (curTime - fileTime > 1800 &&
				((i->path().string().find("_include_light.xml") !=std::string::npos) ||
						(i->path().string().find("_include_table.xml") !=std::string::npos) ||
						(i->path().string().find("_Test.xml") !=std::string::npos) ||
						(i->path().string().find("_include_base_assets.xml") !=std::string::npos) ||
						(i->path().string().find("_include_base.xml") !=std::string::npos) ||
						(i->path().string().find("_include_object_assets.xml") !=std::string::npos) ||
						(i->path().string().find("_include_object.xml") !=std::string::npos))
		){
			try{
				boost::filesystem::remove_all(i->path());
			}
			catch(const std::exception&)
			{}
		}
	}

	// Remove all files from the tmp folder
	boost::filesystem::path p2("./tmp/data");

	// Go over the files and delete them one by one.
	for (auto i = boost::filesystem::directory_iterator(p2); i != boost::filesystem::directory_iterator(); i++)
	{
		time_t fileTime = boost::filesystem::last_write_time(i->path());
		if (curTime - fileTime > 1800)
		{
			try{
				boost::filesystem::remove_all(i->path());
			}
			catch(const std::exception&)
			{}
		}
	}
}

}

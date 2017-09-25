#include <util/util.h>
#include <util/Connector.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <stdio.h>
#include <boost/filesystem.hpp>

#define PI 3.141592

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
    char baseIdFile[1000], oldAssetFile[1000], oldBaseAssetFile[1000], newAssetFile[1000],
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

	// Create random parameters
	float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	r = r / 2 + 0.5;  // random number between 0.5 and 1;
	sprintf(tmp, "friction=\"%f 0.005 0.0001\"", r);

	// Replace friction
	replaceAll(objectStr, std::string("friction=\"\""), std::string(tmp));

	// Create random density
	int density = rand()%500;
	density = density + 100;
	sprintf(tmp, "density=\"%d\"", density);

	// Replace density
	replaceAll(objectStr, std::string("density=\"\""), std::string(tmp));

	// Create random orientation
	r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
	float orientation = PI * 2 * (r - 0.5);
	sprintf(tmp, "euler=\"0 0 %f\"", orientation);
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
	z = z + 1;
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
	intensity = intensity/2 + 0.25; // [0.25 - 0.75]
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
    char tmpStr[1000], modelPrefix[1000];
    bool uploadSuccess = true;

    // Obtain a filename prefix for this specific object
    strcpy(modelPrefix, base);
    strcat(modelPrefix, "/");
    strcat(modelPrefix, planner->fileId);
    strcat(modelPrefix, "_");

    // Upload all files
    strcpy(tmpStr, modelPrefix);
    strcat(tmpStr, "Test.xml");
    uploadSuccess = uploadSuccess && Connector::UploadFile(tmpStr);
    if (boost::filesystem::exists(tmpStr))
    	boost::filesystem::remove(tmpStr);
    strcpy(tmpStr, modelPrefix);
    strcat(tmpStr, "include_light.xml");
    uploadSuccess = uploadSuccess && Connector::UploadFile(tmpStr);
    if (boost::filesystem::exists(tmpStr))
    	boost::filesystem::remove(tmpStr);
    strcpy(tmpStr, modelPrefix);
    strcat(tmpStr, "include_table.xml");
    uploadSuccess = uploadSuccess && Connector::UploadFile(tmpStr);
    if (boost::filesystem::exists(tmpStr))
    	boost::filesystem::remove(tmpStr);
    strcpy(tmpStr, modelPrefix);
    strcat(tmpStr, "include_object.xml");
    uploadSuccess = uploadSuccess && Connector::UploadFile(tmpStr);
    if (boost::filesystem::exists(tmpStr))
    	boost::filesystem::remove(tmpStr);
    strcpy(tmpStr, modelPrefix);
    strcat(tmpStr, "include_object_assets.xml");
    uploadSuccess = uploadSuccess && Connector::UploadFile(tmpStr);
    if (boost::filesystem::exists(tmpStr))
    	boost::filesystem::remove(tmpStr);
    strcpy(tmpStr, modelPrefix);
    strcat(tmpStr, "include_base.xml");
    uploadSuccess = uploadSuccess && Connector::UploadFile(tmpStr);
    if (boost::filesystem::exists(tmpStr))
    	boost::filesystem::remove(tmpStr);
    strcpy(tmpStr, modelPrefix);
    strcat(tmpStr, "include_base_assets.xml");
    uploadSuccess = uploadSuccess && Connector::UploadFile(tmpStr);
    if (boost::filesystem::exists(tmpStr))
    	boost::filesystem::remove(tmpStr);

    uploadSuccess = uploadSuccess && Connector::UploadFile(planner->logFile);
    uploadSuccess = uploadSuccess && Connector::UploadFile(planner->rgbFile);
    uploadSuccess = uploadSuccess && Connector::UploadFile(planner->depthFile);
    uploadSuccess = uploadSuccess && Connector::UploadFile(planner->trajectoryFile);
    uploadSuccess = uploadSuccess && Connector::UploadFile(planner->pointFile);
    uploadSuccess = uploadSuccess && Connector::UploadFile(planner->debugLogFile);
    uploadSuccess = uploadSuccess && Connector::UploadFile(planner->resultFile);

    // Remove log file under all circumstances
    if (boost::filesystem::exists(planner->logFile))
    	boost::filesystem::remove(planner->logFile);

    // delete tmp folder
    if(uploadSuccess && boost::filesystem::exists(planner->baseFolder))
    {
       boost::filesystem::remove_all(planner->baseFolder);
    }
}

}

#include <util/util.h>
#include <util/Connector.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <boost/filesystem.hpp>

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
std::string CreateXMLs(const char * base, GraspPlanner * planner, int objectId, int baseId, int runCount){
	// Templates for model, base and assets files are already there.
	// We just need to modify them by adding the variations.

	// Allocate space for model xml.
	char modelXMLStr[10000];
	modelXMLStr[0] = 0;
	sprintf(modelXMLStr, "<mujoco model=\"BHAM\">\n    <include file=\"include_BHAM.xml\"/>\n    <include file=\"include_assets.xml\"/>\n");

    // Manage file names
    char baseIdFile[1000], classIdFile[1000], oldAssetFile[1000], oldBaseAssetFile[1000], newAssetFile[1000],
	newBaseAssetFile[1000], oldObjectFile[1000], oldBaseFile[1000], newObjectFile[1000], newBaseFile[1000],
	tmp[1000], tmpXML[1000], lightFile[1000], tableFile[1000], modelPrefix[1000], tmpModelPrefix[1000];

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

    // Obtain a temp filename prefix for this specific object
    strcpy(tmpModelPrefix, base);
    strcat(tmpModelPrefix, "/tmp");

    // Create tmp folder if it doesn't exist
    std::cout.flush();
    if (!boost::filesystem::exists(tmpModelPrefix))
    	boost::filesystem::create_directories(tmpModelPrefix);

    // Set prefix
    strcat(tmpModelPrefix, "/");
    strcat(tmpModelPrefix, planner->fileId);
    strcat(tmpModelPrefix, "_include_object");

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
    	planner->minPointZ = -0.3; // Table is at -0.35, each base is about 9 cms.
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

	// Set friction upper limit.
	float frictionLowerLimit = 0.5;
	float frictionUpperLimit = 1;

	// Create random scale based on the object type.
	float xScale = RF/2+0.75;
	float yScale = xScale;
	float zScale = RF/2+0.75;
	float eulerx = 0, eulery = 0, eulerz = RF * 2 * M_PI;
	bool lowerObject = true;
	char lowerStr[20];
	strcpy(lowerStr, "pos=\"0 0 -0.2\"");

	switch(classId)
	{
	case 0: // Bottle
		zScale = RF/2+0.8;
		yScale = RF*0.2+1;
		if (objectId == 4)
		{
			yScale = RF*0.2+0.5;
		}
		xScale = yScale;
		break;
	case 1: // Bowl
		xScale = RF*0.38+0.65; // make it all proportional
		yScale = xScale;
		zScale = xScale;
		lowerObject = false;
		break;
	case 2: // Cup
		zScale = RF/2+1; // longer
		yScale = RF*0.6+0.6;
		xScale = yScale;
		lowerObject = false;
		break;
	case 3: // Fork
		lowerObject = false;
		break; // default
	case 4: // Pan
		xScale = RF/2+0.5;
		yScale = xScale;
		zScale = RF/2+1;
		frictionLowerLimit = 0;
		frictionUpperLimit = 0.5;
		lowerObject = false;
		break;
	case 5: // Jug
		xScale = RF*0.3+0.7; // slightly smaller
		yScale = xScale;
		zScale = xScale;
		lowerObject = false;
		break;
	case 6: // Knife
		zScale = RF*0.3+0.85; // longer
		yScale = RF*0.2 + 0.9;
		xScale = yScale;
		lowerObject = false;
		break; // default
	case 7: // Mug
		xScale = RF*0.1+0.7; // make it all proportional
		yScale = xScale;
		zScale = xScale;
		eulerx = RF * M_PI;
		eulery = RF * M_PI;
		frictionLowerLimit = 0;
		frictionUpperLimit = 0.5;
		break;
	case 8: //Plate
		xScale = RF*0.5+0.75; // make it all proportional
		yScale = xScale;
		zScale = xScale;
		lowerObject = false;
		break;
	case 9: // Scissors
		xScale = RF*0.5+0.75; // make it all proportional
		yScale = xScale;
		zScale = xScale;
		lowerObject = false;
		break; //
	case 10: // Shaker
		xScale = RF*0.2+0.9; // make it all proportional
		yScale = xScale;
		zScale = xScale;
		break;
	case 11: //Spatula
		lowerObject = false;
		break; // default
	case 12: // Spoon
		lowerObject = false;
		break; // default
	case 13: // Teacup
		xScale = RF*0.4+0.9; // make it all proportional, slightly bigger
		yScale = xScale;
		zScale = xScale;
		frictionLowerLimit = 0;
		frictionUpperLimit = 0.5;
		break;
	case 14: // Teapot
		xScale = RF*0.3+0.8; // slightly smaller
		yScale = xScale;
		zScale = xScale;
		lowerObject = false;
		break;
	}

	// Scale the object randomly.
	std::ifstream tAsset(newAssetFile);
	std::string assetStr((std::istreambuf_iterator<char>(tAsset)),
					 std::istreambuf_iterator<char>());
	tAsset.close();

	// Replace scale
	sprintf(tmp, "<mesh scale=\"%f %f %f\" ", xScale, yScale, zScale);
	replaceAll(assetStr, std::string("<mesh "), std::string(tmp));

	// Write light string.
	std::ofstream tAssetOut(newAssetFile);
	tAssetOut<<assetStr;
	tAssetOut.close();

	// For each run, we create a different object with varying friction and weight.
	for (int runItr = 0; runItr<runCount; runItr++)
	{
		// Create random parameters
		float friction = RF * (frictionUpperLimit - frictionLowerLimit) + frictionLowerLimit;

		// Get the right object file.
		boost::filesystem::copy_file(oldObjectFile, newObjectFile, boost::filesystem::copy_option::overwrite_if_exists);

		// Modify model file.
		std::ifstream t(newObjectFile);
		std::string objectStr((std::istreambuf_iterator<char>(t)),
						 std::istreambuf_iterator<char>());

		// Replace friction
		sprintf(tmp, "friction=\"%f %f %f\"", friction, friction*0.005, friction*0.0001);
		replaceAll(objectStr, std::string("friction=\"\""), std::string(tmp));

		// Create random mass, depending on the object type.
		float baseWeights[] = {30, 50, 30, 40, 150, 70, 50, 250, 40, 50, 100, 40, 40, 150, 500};
		float addedWeights[] = {40, 350, 300, 40, 300, 80, 100, 100, 80, 100, 60, 40, 40, 100, 300};
		int baseWeight = baseWeights[classId];
		int addedWeight = addedWeights[classId];
		float mass = (float)(rand()%addedWeight + baseWeight);
		mass = mass/1000;
		sprintf(tmp, "mass=\"%f\"", mass);

		// Replace mass
		replaceAll(objectStr, std::string("mass=\"\""), std::string(tmp));

		// Create random orientation
		sprintf(tmp, "euler=\"%f %f %f\"", eulerx, eulery, eulerz);
		replaceAll(objectStr, std::string("euler=\"\""), std::string(tmp));

		// Lower the object's initial position
		if (lowerObject)
		{
			replaceAll(objectStr, std::string("pos=\"0 0 0\""), std::string(lowerStr));
		}

		// Create random object colour
		float red = ((float)(rand()%255))/255.0, green = ((float)(rand()%255))/255.0, blue = ((float)(rand()%255))/255.0;
		sprintf(tmp, "rgba=\"%f %f %f 1\"", red, green, blue);
		replaceAll(objectStr, std::string("rgba=\"\""), std::string(tmp));

		// Write object file.
		t.close();
		std::ofstream tOutObject(newObjectFile);
		tOutObject<<objectStr;
		tOutObject.close();

		// Finally, move the file into a sub-folder so that we do not clutter the workspace.
		char tmp[1000];
		strcpy(tmp, tmpModelPrefix);
		strcat(tmp, std::to_string(runItr).c_str());
		strcat(tmp, ".xml");
		boost::filesystem::rename(newObjectFile, tmp);
	}

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
		float red = ((float)(rand()%255))/255.0, green = ((float)(rand()%255))/255.0, blue = ((float)(rand()%255))/255.0;
		sprintf(tmp, "rgba=\"%f %f %f 1\"", red, green, blue);
		replaceAll(baseStr, std::string("rgba=\"\""), std::string(tmp));

		// Write table string.
		std::ofstream tBaseOut(newBaseFile);
		tBaseOut<<baseStr;
		tBaseOut.close();
	}

	// Return name of the entry file.
	return outputStr;
}

void UploadFiles(const char * base, GraspPlanner * planner, int objectId, int baseId){

	std::cout<<"BASE FOLDER:"<<planner->baseFolder<<std::endl;
	char viewFolder[1000], imageFolder[1000], dataFile[1000], trjFile[1000];
	strcpy(viewFolder, planner->baseFolder);
	strcat(viewFolder, "/views/");
	strcpy(imageFolder, planner->baseFolder);
	strcat(imageFolder, "/images/");
	if (!boost::filesystem::exists(imageFolder))
		boost::filesystem::create_directories(imageFolder);

	int ctr = 0;
	// First, we need to prepare the grasp data and images.
	// We will create a bin file that includes object class,
	// object id, grasp output (success, stability) and
	// grasp parameters (wrist, joint).

	strcpy(dataFile, planner->baseFolder);
	strcat(dataFile, "data.bin");
	strcpy(trjFile, planner->baseFolder);
	strcat(trjFile, planner->fileId);
	strcat(trjFile, ".trj");
	FILE * trjFP1 = fopen(planner->trajectoryFile, "rb");
	FILE * trjFP2 = fopen(trjFile, "wb");
	int numberOfGrasps, graspType, wpCount;
	float likelihood, buffer[1000];
	fread(&numberOfGrasps, 4, 1, trjFP1);

	FILE * dataFP = fopen(dataFile, "wb");

	for (int i = 0; i<planner->numberOfGrasps; i++)
	{
		// Read original trajectory data
		fread(&likelihood, 4, 1, trjFP1);
		fread(&graspType, 4, 1, trjFP1);
		fread(&wpCount, 4, 1, trjFP1);
		fread(buffer, 4, 27*wpCount, trjFP1);

		// If no trials have been performed, move on.
		if (!planner->resultArr[i].counter)
			continue;

		// Write original trajectory data
		fwrite(&likelihood, 4, 1, trjFP2);
		fwrite(&graspType, 4, 1, trjFP2);
		fwrite(&wpCount, 4, 1, trjFP2);
		fwrite(buffer, 4, 27*wpCount, trjFP2);

		// Get image from views and save into images folder
		std::string im = std::string(viewFolder) + std::string(std::to_string(planner->resultArr[i].viewId)) + std::string(".jpg");
		std::string destIm = std::string(imageFolder) + std::string(std::to_string(ctr)) + std::string(".jpg");
		boost::filesystem::copy_file(im, destIm, boost::filesystem::copy_option::overwrite_if_exists);

		std::string imDepth = std::string(viewFolder) + std::string(std::to_string(planner->resultArr[i].viewId)) + std::string(".png");
		std::string destImDepth = std::string(imageFolder) + std::string(std::to_string(ctr)) + std::string(".png");
		boost::filesystem::copy_file(imDepth, destImDepth, boost::filesystem::copy_option::overwrite_if_exists);

		// Get object id, grasp params, output parameters etc print in a file.
		float success = planner->resultArr[i].successProbability/(double)planner->resultArr[i].counter;
		float stabilityArr[4];
		stabilityArr[0] = planner->resultArr[i].x1/(double)planner->resultArr[i].counter;
		stabilityArr[1] = planner->resultArr[i].r1/(double)planner->resultArr[i].counter;
		stabilityArr[2] = planner->resultArr[i].x2/(double)planner->resultArr[i].counter;
		stabilityArr[3] = planner->resultArr[i].r2/(double)planner->resultArr[i].counter;
		fwrite(&success, 4, 1, dataFP); // Writing success probability
		fwrite(stabilityArr, 4, 4, dataFP); // Writing stability values
		std::vector<float> tmpVec = planner->graspParams[i];
		float * tmpArr = new float[tmpVec.size()];
		for (int k = 0; k<tmpVec.size(); k++)
			tmpArr[k] = planner->graspParams[i][k];
		fwrite(tmpArr, 4, tmpVec.size(), dataFP); // Writing grasp parameters

		// Increase counter.
		ctr++;
	}
	fclose(dataFP);
	fclose(trjFP1);
	fclose(trjFP2);

	if (system(NULL)) puts ("Ok");
    	else exit (EXIT_FAILURE);

	// Remove views folder and unnecessary files
	if (boost::filesystem::exists(viewFolder))
		boost::filesystem::remove_all(viewFolder);
	if (boost::filesystem::exists(planner->debugLogFile))
		boost::filesystem::remove(planner->debugLogFile);
	if (boost::filesystem::exists(planner->rgbFile))
		boost::filesystem::remove(planner->rgbFile);
	if (boost::filesystem::exists(planner->depthFile))
		boost::filesystem::remove(planner->depthFile);

	// Write object id to a file
	char objectIdFile[1000];
	strcpy(objectIdFile, planner->baseFolder);
	strcat(objectIdFile, "/objectId.txt");
	FILE * f = fopen(objectIdFile, "w");
	fprintf(f, "%d\n", objectId);
	fclose(f);

    // Compress the folder
	char command[1000], zipFile[1000];
	strcpy(command, "cd ./tmp/data/");
	strcat(command, planner->fileId);
	strcat(command, " && zip -r ");
	strcat(command, planner->fileId);
	strcat(command, ".zip .");
	strcpy(zipFile, "./tmp/data/");
	strcat(zipFile, planner->fileId);
	strcat(zipFile, "/");
	strcat(zipFile, planner->fileId);
	strcat(zipFile, ".zip");

	// Call zip command
	system(command);

    // Create temp prefix
	char dropboxZipFile[1000];
    strcpy(dropboxZipFile, planner->dropboxFolder);
    strcat(dropboxZipFile, "/upload/");
    strcat(dropboxZipFile, planner->fileId);
    strcat(dropboxZipFile, ".zip");

    // Upload file
    bool uploadSuccess = Connector::UploadFile(zipFile);
    if (!uploadSuccess && boost::filesystem::exists(zipFile))
    {
        std::cout<<"Could not upload. Copying "<<zipFile<<" to "<<dropboxZipFile<<"."<<std::endl;
        boost::filesystem::copy_file(zipFile, dropboxZipFile, boost::filesystem::copy_option::overwrite_if_exists);
        boost::filesystem::remove(zipFile);
    }

    // delete tmp folder
    if(boost::filesystem::exists(planner->baseFolder))
    {
       boost::filesystem::remove_all(planner->baseFolder);
    }
}

void DistributePoints(const char * dropboxBase){
	char tmpStr[1000];
	strcpy(tmpStr, dropboxBase);
	strcat(tmpStr, "/points");

	// No such folder? Return.
	if (!boost::filesystem::is_directory(tmpStr))
		return;

	time_t curTime = time(NULL);
	std::vector<std::string> emptyDirs;

	// Create folders for up to 6 servers, if they don't exist
	for (int i = 0; i<6; i++)
	{
		char tmp[1000];
		strcpy(tmp, dropboxBase);
		strcat(tmp, "/server/");
		char str[10];
		sprintf(str, "%d", i);
		strcat(tmp, str);

		bool dirEmpty = true;
		// If directory doesn't exist, create. If it does, check if it has any pcd files in it.
		if (!boost::filesystem::is_directory(tmp))
				boost::filesystem::create_directories(tmp);
		else{
			// If this directory is empty (no point files), mark it empty.
			boost::filesystem::path p3(tmp);

			// Check if this folder has any pcd files in it
			for (auto k = boost::filesystem::directory_iterator(p3); k != boost::filesystem::directory_iterator(); k++)
			{
				if ((k->path().string().find(".pcd") !=std::string::npos))
				{
					// Mark directory full
					dirEmpty = false;
					break;
				}
			}
		}

		// If no files, mark it empty
		if (dirEmpty)
			emptyDirs.push_back(std::string(tmp));
	}

	// Shuffle emptyDirs
	if (emptyDirs.size() > 0)
		std::random_shuffle ( emptyDirs.begin(), emptyDirs.end() );

	// Get file list and ages in seconds
	boost::filesystem::path p(tmpStr);
	char tmp[1000];
	std::vector<time_t> ages;
	std::vector<std::string> filePaths;

	// Check if this folder has any pcd files in it
	for (auto k = boost::filesystem::directory_iterator(p); k != boost::filesystem::directory_iterator(); k++)
	{
		if ((k->path().string().find(".pcd") !=std::string::npos))
		{
			time_t fileTime = boost::filesystem::last_write_time(k->path());
			ages.push_back(curTime - fileTime);
			filePaths.push_back(k->path().string());
		}
	}

	// Assign point files into empty server folders
	while(true)
	{
		// No empty dirs: break
		if (!emptyDirs.size() || !filePaths.size())
			break;

		// Find oldest file
		std::string oldestFile = filePaths[0];
		time_t age = ages[0];
		int index = 0;

		// Check if this folder has any pcd files in it
		for (int k = 1; k<ages.size(); k++)
		{
			if (ages[k] > age)
			{
				index = k;
				age = ages[k];
				oldestFile = filePaths[k];
			}
		}

		std::cout<<"Assigning " << oldestFile << " to server " << emptyDirs[0].c_str() << std::endl;
		// Assign the point file to the first empty directory
		boost::filesystem::path pFile(oldestFile.c_str());
		char newFile[1000];
		strcpy(newFile, emptyDirs[0].c_str());
		strcat(newFile, "/");
		strcat(newFile, pFile.filename().c_str());
		boost::filesystem::copy_file (oldestFile.c_str(), newFile, boost::filesystem::copy_option::overwrite_if_exists);
		boost::filesystem::remove(oldestFile.c_str());

		// Remove entries
		emptyDirs.erase(emptyDirs.begin());
		ages.erase(ages.begin() + index);
		filePaths.erase(filePaths.begin() + index);
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
	bool uploadFlag = true;
	char tmp[1000];

	// Take the first file you see.
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		if (i->path().extension().string() == ".zip")
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

void RemoveOldPoints(const char * dropboxBase){
	char tmpStr[1000];
	strcpy(tmpStr, dropboxBase);
	strcat(tmpStr, "/points");

	// No such folder? Return.
	if (!boost::filesystem::is_directory(tmpStr))
		return;

	// Try to upload all files under the $dropboxBase$/upload folder to the system.
	boost::filesystem::path p(tmpStr);
	bool uploadFlag = true;
	char tmp[1000];
	time_t curTime = time(NULL);

	// Take the first file you see.
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		time_t fileTime = boost::filesystem::last_write_time(i->path());
		if (curTime - fileTime > 360){
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

	char tmpModelFolder[1000];
	strcpy(tmpModelFolder, modelFolder);
	strcat(tmpModelFolder, "/tmp");

	// Remove all files from the temporary model folder
	if (boost::filesystem::exists(tmpModelFolder))
	{
		boost::filesystem::path p3(tmpModelFolder);

		// Go over the files and delete them one by one.
		for (auto i = boost::filesystem::directory_iterator(p3); i != boost::filesystem::directory_iterator(); i++)
		{
			time_t fileTime = boost::filesystem::last_write_time(i->path());
			if (curTime - fileTime > 1800 &&
					((i->path().string().find(".xml") !=std::string::npos)))
			{
				try{
					boost::filesystem::remove_all(i->path());
				}
				catch(const std::exception&)
				{}
			}
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

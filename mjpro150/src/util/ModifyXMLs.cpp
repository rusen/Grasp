#include <util/util.h>
#include <string>
#include <fstream>
#include <streambuf>
#include <iostream>

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

void ModifyXMLs(const char * base, int objectId, int baseId){
	// The assets and object files for each object is already there.
	// We just need to modify them by adding the variations.

	// Get base folder.
	char objectFile[1000], baseFile[1000], lightFile[1000], tableFile[1000], tmp[100];
	strcpy(objectFile, base);
	strcat(objectFile, "/include_object.xml");
	strcpy(baseFile, base);
	strcat(baseFile, "/include_base.xml");
	strcpy(tableFile, base);
	strcat(tableFile, "/include_table.xml");
	strcpy(lightFile, base);
	strcat(lightFile, "/include_light.xml");

	// Start with the object
	std::ifstream t(objectFile);
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
	std::ofstream tOut(objectFile);
	tOut<<objectStr;
	tOut.close();

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
	x = (x/2) - 0.25;
	y = (static_cast <float> (rand()) / static_cast <float> (RAND_MAX));
	y = (y/2) - 0.25;
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

	// Finally, we fix the base colour.
	std::ifstream tBase(baseFile);
	std::string baseStr((std::istreambuf_iterator<char>(tBase)),
	                 std::istreambuf_iterator<char>());
	tBase.close();
	red = ((float)(rand()%255))/255.0, green = ((float)(rand()%255))/255.0, blue = ((float)(rand()%255))/255.0;
	sprintf(tmp, "rgba=\"%f %f %f 1\"", red, green, blue);
	replaceAll(baseStr, std::string("rgba=\"\""), std::string(tmp));

	// Write table string.
	std::ofstream tBaseOut(baseFile);
	tBaseOut<<baseStr;
	tBaseOut.close();
}


}

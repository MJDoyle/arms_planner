#ifndef MODEL_LOADER_HPP
#define MODEL_LOADER_HPP

#include "assembler/MeshFunctions.hpp"

class Part;

class Assembly;

class ModelLoader {

    public:

        //Load a .step file with OpenCascade and generate a target assembly
        static std::shared_ptr<Assembly> loadModel(const std::string& filename);

    private:

        //Load the individual parts from the .step file
        static std::vector<std::shared_ptr<Part>> loadParts(const std::string& filename);

        static std::string GetShapeName(const TDF_Label& label);

        static void RecurrentAddPart(const TDF_Label& label, const Handle(XCAFDoc_ShapeTool)& shapeTool, std::vector<std::shared_ptr<Part>>& parts, int level);

        static int next_id_;
};

#endif  // MODEL_LOADER_HPP
#ifndef NODE_H
#define NODE_H
#include <string>
#include <vector>
#include <map>
#include <limits>
#include <iostream>
#include <queue>
#include "tinyxml.h"
#include <sstream>

void split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> split(const std::string &s, char delim);

class node;//forward declaration for namespace
namespace aixml
{
    node& openDocument(std::string filename);//!< generates xml-tree of file, returns reference to the root-node
    bool saveDocument(node& root,int toolLevel=0);//!< saves the XML-tree to the file, throws error if toolLevel is smaller than the ToolLevel-attribute in a modified element
    void closeDocument(node& root);//!< delete xml tree
    bool checkSchema(node& schemaXML,node& xmlToBeTested);//!<check weather an XML-File ist path compatible with another (schema)XML
    void applySchema(node& schemaXML,node& xmlToBeUpdated);//!<applies scheme to xmlToBeUpdated, empty nodes get added to xmlToBeUpdated if missing
    node getSingleNode(std::string filename,std::string s,int depth=std::numeric_limits<int>::max());
}

class node
{
private:
    enum class valueType {UNDEFINED,STRING,DOUBLE};//INT,BOOL
public:

    node();
    node(std::string name, bool writeToXml=true);
    node(std::string name, std::string value, bool writeToXml=true);
    node(std::string name, node* parent, bool writeToXml=true);
    node(std::string name, std::string value, node* parent, bool writeToXml=true);
    node(const node& nodeToCopy);
    virtual ~node();

    friend node& aixml::openDocument(std::string filename);
    friend bool aixml::saveDocument(node& root,int toolLevel);
    friend void aixml::closeDocument(node& root);
    friend bool aixml::checkSchema(node& schemaXML,node& xmlToBeTested);
    friend void aixml::applySchema(node& schemaXML,node& xmlToBeUpdated);
    node getSingleNode(std::string filename,std::string s,int depth);

    std::string getName();
    node& at(std::string s,int depth=std::numeric_limits<int>::max());//!< returns a reference to node matching the subPath, if no node is found throws error
    node* find(std::string s,int depth=std::numeric_limits<int>::max());//!< returns a pointer to node matching the subPath, if no node is found returns 0=false
    std::vector<node*> getVector(std::string subPath,int depth=1);//!< returns a vector with nodes matching the subPath
    std::vector<double> getDoubleVector(std::string subPath,int depth=1);//!< returns a vector with doubles of elements matching the subPath
    std::vector<int> getIntVector(std::string subPath,int depth=1);//!< returns a vector with integers of elements matching the subPath
    std::vector<bool> getBoolVector(std::string subPath,int depth=1);//!< returns a vector with booleans of elements matching the subPath
    std::vector<std::string> getStringVector(std::string subPath,int depth=1);//!< returns a vector with strings of elements matching the subPath

    bool getBoolAttrib(std::string name);//!< returns an attribute with the given name as boolean
    int getIntAttrib(std::string name);//!< returns an attribute with the given name as integer
    double getDoubleAttrib(std::string name);//!< returns an attribute with the given name as double
    std::string getStringAttrib(std::string name);//!< returns an attribute with the given name as string
    bool hasAttrib(std::string name);//!< returns an attribute with the given name as string
    template<class T> void addAttrib(std::string name, T value); //!< Add an attribute to the node, keeps the old value if attribute already exists
    template<class T> void setAttrib(std::string name, T value,bool addIfNotExistent=true); //!< Sets an attribute, depending on addIfNotExistent a new attribute can be added
    void deleteAttrib(std::string name); //!< Deletes an attribute from the node

    node& appendChild(std::string name, bool writeToXML=false);//!< Constructs an new node and appends to this node as child, if writeToXML is true, new node appears in XML-File
    node& appendChild(node* new_node); //!< Appends an existing node to this node as child
    node& appendChild(const node& nodeToBeInserted); //!< Copies and appends an existing node to this node as child
    void deleteChild(std::string name); //!< Deletes the child and the full subtree
    void deleteChild(node* old_node); //!< Recursive function get called for deleting the subtree from the child on
    void deleteChildren(); //!< Deletes all children including their subtrees
    node& duplicate(node* new_parent=nullptr);//!< duplicates this node and by default appends to parent-element after this node, returns reference to new node
    node& duplicateEmpty(node* new_parent=nullptr);//!< duplicates this node, fills new node with zeros and by default appends to parent-element after this node, returns reference to new node
    node& replaceChild(std::string name, const node& nodeToBeInserted, bool createIfNotExistent=false);//!< creates copy of nodeToBeInserted and replaces existing node with name=name (including identifiers) old node gets deleted
    node& insertAfterChild(std::string name, const node& nodeToBeInserted);//!< creates copy of nodeToBeInserted and inserts after node with name=name (including identifiers)
    node& insertBeforeChild(std::string name, const node& nodeToBeInserted);//!< creates copy of nodeToBeInserted and inserts before node with name=name (including identifiers)

    void setToZero(bool setAttributesToZero=false);//!< sets everything to zero, if setAttributesToZero is set, also attributes get set to zero
    //conversion-operators
    operator double();
    operator std::string();
    node& operator [](std::string s);

    //overloaded operators
    friend std::ostream& operator<< (std::ostream& stream, node& n) ;

    void operator=(const double d);
    void operator=(const int i);
    void operator=(const bool b);
    void operator=(const std::string s);
    void operator=(const char* s);


    friend std::string operator+(std::string value, node& n);
    friend std::string operator+(node& n,std::string value);
    friend double operator+(double value, node& n);
    friend double operator+(node& n, double value);
    friend double operator-(double value, node& n);
    friend double operator-(node& n, double value);
    friend double operator/(double value, node& n);
    friend double operator/(node& n, double value);
    friend double operator*(double value, node& n);
    friend double operator*(node& n, double value);
    double operator*(node& n);
    double operator/(node& n);
    double operator+(node& n);
    double operator-(node& n);

    double arithmetic_operation(node& n,char c);

    friend double checkBoundaries(node& value,double minValue, double maxValue);
    friend double checkBoundaries(node& value,double minValue,bool includeLeft, double maxValue, bool includeRight);

    std::string getFullPath(bool includingIdentifier=true);

    static std::vector<std::string> input;
    static std::vector<std::string> output;


protected:

private:
    std::string name;
    std::map<std::string,std::string> attributes;
    node* parent;
    std::vector<node*> children;
    std::string undefinedValue;
    std::string stringValue;
//    int intValue;
    double doubleValue;
//    bool boolValue;
    valueType type;
    void addNodeToDom(TiXmlNode* tiNode);
    std::string getResult();
    static void addNodeToDoc(node* treeNode,TiXmlElement* docNode,int toolLevel);
    bool writeToXml;
    bool modified;
    void addToInputOutput(bool isInput=true);
    void init(std::string name="", bool writeToXml=true, std::string value="", node* parent=nullptr); //!< Initialisiert die Attribute mit gegebenen Werten
    std::vector<node*> findVector(std::string subPath, bool findAll, int depth);
};

template<class T> void node::addAttrib(std::string name, T value) //!< Add an attribute to the node
{
    std::stringstream ss;
    ss<<value;
    attributes.emplace(name, ss.str());
}
template<class T> void node::setAttrib(std::string name, T value,bool addIfNotAxistent) //!< Add an attribute to the node
{
    std::stringstream ss;
    ss<<value;
    if(addIfNotAxistent)
    {
        attributes[name]=ss.str();
    }
    else if(attributes.count(name))
    {
        attributes[name]=ss.str();
    }

}

#endif // NODE_H

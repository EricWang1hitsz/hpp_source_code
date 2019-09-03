// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation-urdf.
// hpp-manipulation-urdf is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-urdf is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-urdf. If not, see <http://www.gnu.org/licenses/>.

#ifndef HPP_MANIPULATION_PARSER_HH
# define HPP_MANIPULATION_PARSER_HH

# include <map>
# include <list>
# include <string>
# include <iostream>
# include <tinyxml.h>

# include <boost/function.hpp>

namespace hpp {
  namespace util {
    namespace parser {
      typedef TiXmlElement XMLElement;
      typedef TiXmlDocument XMLDocument;
      typedef TiXmlDeclaration XMLDeclaration;
      typedef TiXmlAttribute XMLAttribute;
      typedef TiXmlNode XMLNode;
      typedef TiXmlText XMLText;
      typedef TiXmlComment XMLComment;

      typedef TiXmlPrinter XMLPrinter;

      class RootFactory;

      /// \defgroup factories
      ///
      /// \brief    Classes used to build object from XML documents.
      ///
      /// See section \ref hpp_manipulation_urdf_extend_sec for more information
      /// about how to extend the parser with factories.

      /// \addtogroup factories
      /// \{

      /// \brief Class that catch XML Parser events for a specific tag and build the corresponding
      /// Object.
      ///
      /// Derive this class if you wish to extend the Parser.
      /// The event callbacks are called in the following order:
      /// \li ObjectFactory::init after having created the object.
      /// \li ObjectFactory::setAttribute for each attribute of the tag.
      /// \li ObjectFactory::finishAttributes after having processed every attribute.
      /// \li ObjectFactory::addTextChild when a child is a text element.
      /// \li ObjectFactory::finishTags when all the children have been parsed.
      /// \li ObjectFactory::finishFile when the file has been fully parsed.
      ///
      /// \note The derived class must have the following construtor
      /// \code
      /// DerivedFactory (ObjectFactory* parent, const XMLElement* element) :
      ///       ObjectFactory (parent, element)
      /// {
      ///   /*
      ///    * Keep in mind that it might be more convenient
      ///    * to build objects in an event callback, when some information
      ///    * has already been parsed.
      ///    */
      /// }
      /// \endcode
      class ObjectFactory {
        public:
          typedef std::list <ObjectFactory*> ObjectFactoryList;

          ObjectFactory (ObjectFactory* parent = NULL, const XMLElement* element = NULL);

	  virtual ~ObjectFactory() {}
          /// \name Events
          /// \{

          /// Called when the object is created.
          /// \return True to continue parsing this tag, False otherwise.
          virtual bool init ();

          /// Called for each attribute.
          /// A few reserved name are automatocally catched. The reserved names are
          /// "name" and "id".
          /// "name" expects a string.
          /// "id" expects an unsigned integer and can be use to define pointers to
          /// elements.
          void setAttribute (const XMLAttribute* attr);

          /// Add Text child.
          virtual void addTextChild (const XMLText* text);

          /// Called when all the attributes have been processed.
          /// \return True to continue parsing this tag, False otherwise.
          virtual bool finishAttributes ();

          /// Called when all the child tags have been processed.
          virtual void finishTags ();

          /// Called when parsing is finished.
          virtual void finishFile ();

          /// \}

          /// \name Write to file
          /// \{

          /// Constructor for writing objects from scratch
          ObjectFactory (const std::string& tagName, ObjectFactory* parent = NULL);

          /// Add an attribute
          void addAttribute (const std::string& name, const std::string& value);

          /// Add this factory as child of the node argument.
          /// Tags are handled throught children so you should add children
          /// before calling this function.
          /// If you factory must write something different from tags (XMLText 
          /// or XMLComment), reimplement method impl_write.
          XMLNode* write (XMLNode* node) const;

          /// \}

          /// \name Accessors
          /// \{

          /// Return tag name of the element is any.
          /// Returns "No element" otherwise.
          std::string tagName () const;

          /// Return the content of the attribute name, or an
          /// empty string.
          std::string name () const;

          /// Check if an attribute was set.
          bool hasAttribute (const std::string& attr) const;

          /// Return a given attributes.
          std::string getAttribute (const std::string& attr) const;

          /// Get a list of ObjectFactory whose tag name is type.
          ObjectFactoryList getChildrenOfType (std::string type);

          /// Get the ObjectFactory whose tag name is type.
          /// \param[out] o Set to the first element of the requested type.
          /// \return true if there was only element of the requested type. false if there are more than one.
          /// \throws std::invalid_argument if no ObjectFactory of the requested type exists.
          bool getChildOfType (std::string type, ObjectFactory*& o);

          /// \}

          /// Set the name.
          /// The default value is the value of the attribute "name"
          /// of the XML tag or an empty string if this does not exist.
          void name (const std::string& n);

          /// See name(const std::string&)
          void name (const char* n);

          /// Cast this class to any child class.
          template <typename T> T* as ()
          {
            return static_cast <T*> (this);
          }

        protected:
          ObjectFactory (ObjectFactory* root);

          ObjectFactory* parent ();

          virtual ObjectFactory* root ();

          bool hasParent () const;

          const XMLElement* XMLelement ();

          virtual void impl_setAttribute (const XMLAttribute* attr);

          virtual void impl_write (XMLElement* element) const;

          void addChild (ObjectFactory* child);

          virtual std::ostream& print (std::ostream& os) const;

        private:
          ObjectFactory* parent_;
          ObjectFactory* root_;
          typedef std::map <std::string, ObjectFactoryList > ChildrenMap;
          ChildrenMap children_;

          const XMLElement* element_;

          typedef std::map <std::string, std::string> AttributeMap;
          AttributeMap attrMap_;
          std::string name_, tagName_;
          int id_;

          friend std::ostream& operator<< (std::ostream&, const ObjectFactory&);
      };
      /// \}

      /// To add a ObjectFactory to the Parser, use:
      /// Parser::addObjectFactory (TagName, create <ObjectFactory>)
      template <typename T>
      ObjectFactory* create (ObjectFactory* parent = NULL, const XMLElement* element = NULL)
      {
        return new T (parent, element);
      }

      /// \brief Parse an XML document
      ///
      /// This class uses the tinyXML library and derived classes of ObjectFactory
      /// to build object from an XML document.
      /// To extend its capabilities, see ObjectFactory.
      class Parser {
        public:
          typedef boost::function <ObjectFactory* (ObjectFactory*, const XMLElement*)>
                    FactoryType;

          /// Constructor
          /// \param defaultFactory The factory used when a tag is not known.
          Parser (FactoryType defaultFactory = create <ObjectFactory>);

          virtual ~Parser ();

          void addObjectFactory (const std::string& tagname, FactoryType factory);

          virtual void parse (const char* xmlString);

          virtual void parseFile (const std::string& filename);

          ObjectFactory* root () const;

        private:
          XMLDocument doc_;
          ObjectFactory* root_;

          bool checkError () const;

          void loadString (const char* xmlstring);

          void parse ();

          void parseElement (const XMLElement* element, ObjectFactory* parent);

          typedef std::map <std::string, FactoryType> ObjectFactoryMap;
          typedef std::pair <std::string, FactoryType> ObjectFactoryPair;
          typedef std::pair <ObjectFactoryMap::iterator, bool> ObjectFactoryInsertRet;
          ObjectFactoryMap objFactoryMap_;
          FactoryType defaultFactory_;

          typedef std::list <ObjectFactory*> ObjectFactoryList;
          ObjectFactoryList objectFactories_;

          std::ostream& print (std::ostream&) const;
          friend std::ostream& operator<< (std::ostream&, const Parser&);
      };

      std::ostream& operator<< (std::ostream&, const ObjectFactory&);
      std::ostream& operator<< (std::ostream&, const Parser&);
    } // namespace parser
  } // namespace manipulation
} // namespace hpp

#endif //  HPP_MANIPULATION_PARSER_HH

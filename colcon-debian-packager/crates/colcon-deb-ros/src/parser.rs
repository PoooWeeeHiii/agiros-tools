//! Package.xml parser

use std::path::Path;
use std::str::FromStr;

use colcon_deb_core::package::{BuildType, Dependencies, Maintainer};
use colcon_deb_core::{Package, Result};
use quick_xml::events::{BytesStart, Event};
use quick_xml::Reader;
use thiserror::Error;

use crate::package_xml::{DependencySpec, PackageDependencies, PackageManifest, Person};

#[derive(Error, Debug)]
pub enum ParseError {
    #[error("XML parsing error: {0}")]
    XmlError(#[from] quick_xml::Error),

    #[error("Invalid UTF-8 in XML: {0}")]
    Utf8Error(#[from] std::str::Utf8Error),

    #[error("Missing required field: {0}")]
    MissingField(String),

    #[error("Invalid package format: {0}")]
    InvalidFormat(String),

    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),

    #[error("XML attribute error: {0}")]
    AttrError(#[from] quick_xml::events::attributes::AttrError),
}

/// Parse a package.xml file
pub fn parse_package_xml(path: &Path) -> Result<Package> {
    let content = std::fs::read_to_string(path)?;
    let manifest = parse_package_manifest(&content).map_err(|e| {
        colcon_deb_core::error::Error::ParseError {
            message: format!("Failed to parse {}: {}", path.display(), e),
        }
    })?;

    Ok(manifest_to_package(manifest, path))
}

/// Parse package.xml content into a manifest
pub fn parse_package_manifest(xml_content: &str) -> Result<PackageManifest> {
    parse_manifest_internal(xml_content)
        .map_err(|e| colcon_deb_core::error::Error::ParseError { message: e.to_string() })
}

fn parse_manifest_internal(xml_content: &str) -> std::result::Result<PackageManifest, ParseError> {
    let mut reader = Reader::from_str(xml_content);
    reader.trim_text(true);

    let mut manifest = PackageManifest {
        name: String::new(),
        version: String::new(),
        description: String::new(),
        maintainers: Vec::new(),
        authors: Vec::new(),
        licenses: Vec::new(),
        urls: Vec::new(),
        build_type: None,
        dependencies: PackageDependencies::default(),
    };

    let mut buf = Vec::new();
    let mut current_element = String::new();
    let mut in_export = false;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Start(ref e)) => {
                let name_bytes = e.name();
                let name = std::str::from_utf8(name_bytes.as_ref())?;
                current_element = name.to_string();

                match name {
                    "export" => in_export = true,
                    "build_depend"
                    | "build_export_depend"
                    | "buildtool_depend"
                    | "buildtool_export_depend"
                    | "exec_depend"
                    | "test_depend"
                    | "doc_depend"
                    | "depend" => {
                        if let Some(dep) = parse_dependency(&mut reader, e)? {
                            add_dependency(&mut manifest.dependencies, name, dep);
                        }
                    }
                    "maintainer" | "author" => {
                        if let Some(person) = parse_person(&mut reader, e)? {
                            match name {
                                "maintainer" => manifest.maintainers.push(person),
                                "author" => manifest.authors.push(person),
                                _ => {}
                            }
                        }
                    }
                    "url" => {
                        if let Some(url) = parse_url(&mut reader, e)? {
                            manifest.urls.push(url);
                        }
                    }
                    _ => {}
                }
            }
            Ok(Event::End(ref e)) => {
                let name_bytes = e.name();
                let name = std::str::from_utf8(name_bytes.as_ref())?;
                if name == "export" {
                    in_export = false;
                }
            }
            Ok(Event::Text(e)) => {
                let text = e.unescape()?.trim().to_string();
                if !text.is_empty() {
                    match current_element.as_str() {
                        "name" => manifest.name = text,
                        "version" => manifest.version = text,
                        "description" => manifest.description = text,
                        "license" => manifest.licenses.push(text),
                        "build_type" if in_export => manifest.build_type = Some(text),
                        _ => {}
                    }
                }
            }
            Ok(Event::Empty(ref e)) => {
                let name_bytes = e.name();
                let name = std::str::from_utf8(name_bytes.as_ref())?;
                if in_export && name == "build_type" {
                    // ROS 2 uses build_type as an empty element with the value as the element name
                    // e.g., <build_type>ament_cmake</build_type>
                    // This is handled in the Text event, not here
                    current_element = "build_type".to_string();
                }
            }
            Ok(Event::Eof) => break,
            Err(e) => return Err(e.into()),
            _ => {}
        }
        buf.clear();
    }

    // Validate required fields
    if manifest.name.is_empty() {
        return Err(ParseError::MissingField("name".to_string()));
    }
    if manifest.version.is_empty() {
        return Err(ParseError::MissingField("version".to_string()));
    }
    if manifest.description.is_empty() {
        return Err(ParseError::MissingField("description".to_string()));
    }
    if manifest.maintainers.is_empty() {
        return Err(ParseError::MissingField("maintainer".to_string()));
    }

    // Expand generic dependencies
    manifest.dependencies.expand_generic_depends();

    Ok(manifest)
}

fn parse_dependency(
    reader: &mut Reader<&[u8]>,
    e: &BytesStart,
) -> std::result::Result<Option<DependencySpec>, ParseError> {
    let mut dep = DependencySpec {
        name: String::new(),
        version_eq: None,
        version_gte: None,
        version_lte: None,
        version_gt: None,
        version_lt: None,
        condition: None,
    };

    // Parse attributes
    for attr in e.attributes() {
        let attr = attr?;
        let key = std::str::from_utf8(attr.key.as_ref())?;
        let value = std::str::from_utf8(&attr.value)?;

        match key {
            "version_eq" => dep.version_eq = Some(value.to_string()),
            "version_gte" => dep.version_gte = Some(value.to_string()),
            "version_lte" => dep.version_lte = Some(value.to_string()),
            "version_gt" => dep.version_gt = Some(value.to_string()),
            "version_lt" => dep.version_lt = Some(value.to_string()),
            "condition" => dep.condition = Some(value.to_string()),
            _ => {}
        }
    }

    // Read the dependency name from text content
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Text(e)) => {
                dep.name = e.unescape()?.trim().to_string();
            }
            Ok(Event::End(_)) => break,
            Ok(Event::Eof) => break,
            _ => {}
        }
    }

    if dep.name.is_empty() {
        Ok(None)
    } else {
        Ok(Some(dep))
    }
}

fn parse_person(
    reader: &mut Reader<&[u8]>,
    e: &BytesStart,
) -> std::result::Result<Option<Person>, ParseError> {
    let mut person = Person { name: String::new(), email: None };

    // Check for email attribute
    for attr in e.attributes() {
        let attr = attr?;
        if attr.key.as_ref() == b"email" {
            person.email = Some(std::str::from_utf8(&attr.value)?.to_string());
        }
    }

    // Read the name from text content
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Text(e)) => {
                person.name = e.unescape()?.trim().to_string();
            }
            Ok(Event::End(_)) => break,
            Ok(Event::Eof) => break,
            _ => {}
        }
    }

    if person.name.is_empty() {
        Ok(None)
    } else {
        Ok(Some(person))
    }
}

fn parse_url(
    reader: &mut Reader<&[u8]>,
    e: &BytesStart,
) -> std::result::Result<Option<crate::package_xml::Url>, ParseError> {
    let mut url = crate::package_xml::Url { url_type: None, url: String::new() };

    // Check for type attribute
    for attr in e.attributes() {
        let attr = attr?;
        if attr.key.as_ref() == b"type" {
            url.url_type = Some(std::str::from_utf8(&attr.value)?.to_string());
        }
    }

    // Read the URL from text content
    let mut buf = Vec::new();
    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Text(e)) => {
                url.url = e.unescape()?.trim().to_string();
            }
            Ok(Event::End(_)) => break,
            Ok(Event::Eof) => break,
            _ => {}
        }
    }

    if url.url.is_empty() {
        Ok(None)
    } else {
        Ok(Some(url))
    }
}

fn add_dependency(deps: &mut PackageDependencies, dep_type: &str, dep: DependencySpec) {
    match dep_type {
        "build_depend" => deps.build_depend.push(dep),
        "build_export_depend" => deps.build_export_depend.push(dep),
        "buildtool_depend" => deps.buildtool_depend.push(dep),
        "buildtool_export_depend" => deps.buildtool_export_depend.push(dep),
        "exec_depend" => deps.exec_depend.push(dep),
        "test_depend" => deps.test_depend.push(dep),
        "doc_depend" => deps.doc_depend.push(dep),
        "depend" => deps.depend.push(dep),
        _ => {}
    }
}

/// Convert a package manifest to the core Package type
fn manifest_to_package(manifest: PackageManifest, path: &Path) -> Package {
    // Determine build type
    let build_type = if let Some(bt) = &manifest.build_type {
        BuildType::from_str(bt).unwrap_or(BuildType::Unknown)
    } else {
        // Infer from dependencies
        if manifest
            .dependencies
            .buildtool_depend
            .iter()
            .any(|d| d.name == "ament_cmake")
        {
            BuildType::AmentCmake
        } else if manifest
            .dependencies
            .buildtool_depend
            .iter()
            .any(|d| d.name == "ament_python")
        {
            BuildType::AmentPython
        } else if manifest
            .dependencies
            .buildtool_depend
            .iter()
            .any(|d| d.name == "cmake")
        {
            BuildType::Cmake
        } else {
            BuildType::Unknown
        }
    };

    // Convert dependencies
    let dependencies = Dependencies {
        build: manifest
            .dependencies
            .build_depend
            .iter()
            .map(|d| d.name.clone())
            .collect(),
        build_export: manifest
            .dependencies
            .build_export_depend
            .iter()
            .map(|d| d.name.clone())
            .collect(),
        exec: manifest
            .dependencies
            .exec_depend
            .iter()
            .map(|d| d.name.clone())
            .collect(),
        test: manifest
            .dependencies
            .test_depend
            .iter()
            .map(|d| d.name.clone())
            .collect(),
        build_tool: manifest
            .dependencies
            .buildtool_depend
            .iter()
            .map(|d| d.name.clone())
            .collect(),
        doc: manifest
            .dependencies
            .doc_depend
            .iter()
            .map(|d| d.name.clone())
            .collect(),
    };

    // Convert maintainers
    let maintainers = manifest
        .maintainers
        .into_iter()
        .map(|p| Maintainer { name: p.name, email: p.email.unwrap_or_default() })
        .collect();

    Package {
        name: manifest.name,
        version: manifest.version,
        description: manifest.description,
        path: path.parent().unwrap_or(path).to_path_buf(),
        maintainers,
        license: manifest.licenses.join(", "),
        build_type,
        dependencies,
    }
}

//! Docker image management

use std::collections::HashMap;

use bollard::image::{BuildImageOptions, CreateImageOptions, ListImagesOptions};
use bollard::models::ImageSummary;
use futures::StreamExt;
use tar::Builder;
use tracing::{debug, info};

use crate::error::{DockerError, Result};
use crate::types::{BuildContext, ImageInfo};

/// Docker image manager
pub struct ImageManager<'a> {
    client: &'a bollard::Docker,
}

impl<'a> ImageManager<'a> {
    /// Create a new image manager
    pub fn new(client: &'a bollard::Docker) -> Self {
        Self { client }
    }

    /// Pull an image from registry
    pub async fn pull(&self, image: &str) -> Result<()> {
        let options = CreateImageOptions { from_image: image, ..Default::default() };

        info!("Pulling Docker image: {}", image);

        let mut stream = self.client.create_image(Some(options), None, None);

        while let Some(info) = stream.next().await {
            match info {
                Ok(output) => {
                    if let Some(status) = output.status {
                        debug!("Pull status: {}", status);
                    }
                    if let Some(error) = output.error {
                        return Err(DockerError::PullFailed {
                            image: image.to_string(),
                            reason: error,
                        });
                    }
                }
                Err(e) => {
                    return Err(DockerError::PullFailed {
                        image: image.to_string(),
                        reason: e.to_string(),
                    });
                }
            }
        }

        info!("Successfully pulled image: {}", image);
        Ok(())
    }

    /// Check if image exists locally
    pub async fn exists(&self, image: &str) -> Result<bool> {
        let filters = HashMap::from([("reference", vec![image])]);
        let options = ListImagesOptions { all: false, filters, ..Default::default() };

        let images = self
            .client
            .list_images(Some(options))
            .await
            .map_err(DockerError::Client)?;

        Ok(!images.is_empty())
    }

    /// Build image from Dockerfile
    pub async fn build(&self, context: &BuildContext, tag: &str) -> Result<()> {
        info!("Building Docker image: {}", tag);

        // Create tar archive with Dockerfile
        let mut tar_data = Vec::new();
        {
            let mut builder = Builder::new(&mut tar_data);

            // Add Dockerfile
            let dockerfile_bytes = context.dockerfile.as_bytes();
            let mut header = tar::Header::new_gnu();
            header
                .set_path("Dockerfile")
                .map_err(|e| DockerError::BuildFailed {
                    reason: format!("Failed to set Dockerfile path: {e}"),
                })?;
            header.set_size(dockerfile_bytes.len() as u64);
            header.set_mode(0o644);
            header.set_cksum();

            builder
                .append(&header, dockerfile_bytes)
                .map_err(|e| DockerError::BuildFailed {
                    reason: format!("Failed to add Dockerfile to tar: {e}"),
                })?;

            builder.finish().map_err(|e| DockerError::BuildFailed {
                reason: format!("Failed to finish tar archive: {e}"),
            })?;
        }

        // Build options
        let mut build_args = HashMap::new();
        for (key, value) in &context.build_args {
            build_args.insert(key.as_str(), value.as_str());
        }

        let mut labels = HashMap::new();
        for (key, value) in &context.labels {
            labels.insert(key.as_str(), value.as_str());
        }

        let options = BuildImageOptions {
            t: tag,
            dockerfile: "Dockerfile",
            buildargs: build_args,
            labels,
            ..Default::default()
        };

        let mut stream = self
            .client
            .build_image(options, None, Some(tar_data.into()));

        while let Some(info) = stream.next().await {
            match info {
                Ok(output) => {
                    if let Some(stream) = output.stream {
                        debug!("Build output: {}", stream.trim());
                    }
                    if let Some(error) = output.error {
                        return Err(DockerError::BuildFailed { reason: error });
                    }
                }
                Err(e) => {
                    return Err(DockerError::BuildFailed { reason: e.to_string() });
                }
            }
        }

        info!("Successfully built image: {}", tag);
        Ok(())
    }

    /// List images
    pub async fn list(&self, filter: Option<&str>) -> Result<Vec<ImageInfo>> {
        let filters = if let Some(f) = filter {
            HashMap::from([("reference", vec![f])])
        } else {
            HashMap::new()
        };

        let options = ListImagesOptions { all: false, filters, ..Default::default() };

        let images = self
            .client
            .list_images(Some(options))
            .await
            .map_err(DockerError::Client)?;

        Ok(images
            .into_iter()
            .map(|img| self.convert_image_summary(img))
            .collect())
    }

    /// Remove an image
    pub async fn remove(&self, image: &str, _force: bool) -> Result<()> {
        info!("Removing Docker image: {}", image);

        self.client
            .remove_image(image, None, None)
            .await
            .map_err(DockerError::Client)?;

        info!("Successfully removed image: {}", image);
        Ok(())
    }

    /// Get image info
    pub async fn inspect(&self, image: &str) -> Result<ImageInfo> {
        let image_data = self
            .client
            .inspect_image(image)
            .await
            .map_err(DockerError::Client)?;

        Ok(ImageInfo {
            id: image_data.id.unwrap_or_default(),
            tags: image_data.repo_tags.unwrap_or_else(Vec::new),
            size: image_data.size.unwrap_or(0),
            created: image_data.created.unwrap_or_default(),
            labels: image_data.config.and_then(|c| c.labels).unwrap_or_default(),
        })
    }

    /// Convert ImageSummary to ImageInfo
    fn convert_image_summary(&self, summary: ImageSummary) -> ImageInfo {
        ImageInfo {
            id: summary.id,
            tags: summary.repo_tags,
            size: summary.size,
            created: summary.created.to_string(),
            labels: summary.labels,
        }
    }

    /// Pull image if it doesn't exist
    pub async fn ensure_image(&self, image: &str) -> Result<()> {
        if !self.exists(image).await? {
            info!("Image {} not found locally, pulling...", image);
            self.pull(image).await?;
        } else {
            debug!("Image {} already exists locally", image);
        }
        Ok(())
    }
}

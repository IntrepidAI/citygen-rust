// This is an asset reader that loads files from anywhere in your filesystem,
// not just from the /assets folder.

use std::path::Path;

use async_fs::File;
use bevy::asset::io::{
    AssetReader, AssetReaderError, AssetSource, AssetSourceId, PathStream, Reader,
};
use bevy::prelude::*;
use bevy::utils::BoxedFuture;

pub struct AssetReaderPlugin;

impl Plugin for AssetReaderPlugin {
    fn build(&self, app: &mut App) {
        app.register_asset_source(AssetSourceId::Default, AssetSource::build().with_reader({
            || Box::new(FileAssetReader)
        }));
    }
}

struct FileAssetReader;

impl AssetReader for FileAssetReader {
    fn read<'a>(
        &'a self,
        path: &'a Path,
    ) -> BoxedFuture<'a, Result<Box<Reader<'a>>, AssetReaderError>> {
        let path = path.to_path_buf();
        Box::pin(async move {
            match File::open(&path).await {
                Ok(file) => {
                    let reader: Box<Reader> = Box::new(file);
                    Ok(reader)
                }
                Err(e) => {
                    if e.kind() == std::io::ErrorKind::NotFound {
                        Err(AssetReaderError::NotFound(path))
                    } else {
                        Err(e.into())
                    }
                }
            }
        })
    }

    fn read_meta<'a>(
        &'a self,
        path: &'a Path,
    ) -> BoxedFuture<'a, Result<Box<Reader<'a>>, AssetReaderError>> {
        Box::pin(async move { Err(AssetReaderError::NotFound(path.to_path_buf())) })
    }

    fn read_directory<'a>(
        &'a self,
        path: &'a Path,
    ) -> BoxedFuture<'a, Result<Box<PathStream>, AssetReaderError>> {
        Box::pin(async move { Err(AssetReaderError::NotFound(path.to_path_buf())) })
    }

    fn is_directory<'a>(
        &'a self,
        path: &'a Path,
    ) -> BoxedFuture<'a, Result<bool, AssetReaderError>> {
        Box::pin(async move { Err(AssetReaderError::NotFound(path.to_path_buf())) })
    }
}

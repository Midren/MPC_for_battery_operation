from pathlib import Path
import typing as t

import torch
import numpy as np
import pandas as pd
import pytorch_lightning as pl
from torch.utils.data import Dataset, DataLoader

class TimeseriesDataset(Dataset):
    '''
    Custom Dataset subclass.
    Serves as input to DataLoader to transform X
      into sequence data using rolling window.
    DataLoader using this dataset will output batches
      of `(batch_size, seq_len, n_features)` shape.
    Suitable as an input to RNNs.

    from: https://www.kaggle.com/tartakovsky/pytorch-lightning-lstm-timeseries-clean-code
    '''
    def __init__(self, X: np.ndarray, y: np.ndarray, seq_len: int = 1):
        self.X = torch.tensor(X).float()
        self.y = torch.tensor(y).float()
        self.seq_len = seq_len

    def __len__(self):
        return self.X.__len__() - (self.seq_len-1)

    def __getitem__(self, index):
        return (self.X[index:index+self.seq_len], self.y[index+self.seq_len-1])

class SwedenLoadDataModule(pl.LightningDataModule):
    def __init__(self, filepath: Path = Path('data/sweden_load_2005_2017.csv'), batch_size: int = 32):
        super().__init__()
        self.filepath = filepath
        df = pd.read_csv(filepath)
        df['cet_cest_timestamp'] = df['cet_cest_timestamp'].apply(lambda x: x.replace(tzinfo=None))
        df = df.rename({'cet_cest_timestamp': 'time', 'SE_load_actual_tso': 'load'}, axis=1)
        self.df = df

    def setup(self, stage=None):
        if stage == 'fit' and self.X_train is not None:
            return
        if stage == 'test' and self.X_test is not None:
            return
        if stage is None and self.X_train is not None and self.X_test is not None:
            return

    def train_dataloader(self) -> DataLoader:
        ...

    def val_dataloader(self) -> t.Union[DataLoader, t.List[DataLoader]]:
        ...

    def test_dataloader(self) -> t.Union[DataLoader, t.List[DataLoader]]:
        ...

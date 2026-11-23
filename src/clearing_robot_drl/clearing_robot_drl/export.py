"""将可信策略权重导出为固定接口的 ONNX。"""
import argparse
import torch
from .core import SAC

def main():
    """! @brief 导出确定性融合策略，输入先验及输出动作均为归一化值。"""
    parser=argparse.ArgumentParser()
    parser.add_argument('weights')
    parser.add_argument('output')
    args=parser.parse_args()
    agent,_,_=SAC.load(args.weights)
    agent.policy.eval()
    torch.onnx.export(agent.policy,(torch.zeros(1,28),torch.zeros(1,2)),args.output,
                      input_names=['observation','prior'],output_names=['action'],opset_version=17,
                      do_constant_folding=True,dynamo=False)

if __name__ == '__main__':
    main()

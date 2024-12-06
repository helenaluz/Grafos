import random
import matplotlib.pyplot as plt
import pyautogui as p
import networkx as nx
import threading


class Grafo:
    def __init__(self, dirigido: bool = False):
        self.vertices = set()
        self.arestas = {}
        self.dirigido = dirigido

    def adicionarVertice(self, vertice: str) -> None:
        """Adiciona um vértice ao grafo."""
        self.vertices.add(vertice)

    def adicionarAresta(self, origem: str, destino: str, peso: float = 1.0) -> None:
        """Adiciona uma aresta com um peso ao grafo."""
        if origem in self.vertices and destino in self.vertices:
            self.arestas[(origem, destino)] = peso
        else:
            raise ValueError("Os vértices da aresta devem estar no conjunto de vértices.")

    def geraGrafo(self) -> nx.Graph:
        """Gera um objeto NetworkX a partir dos vértices e arestas."""
        G = nx.DiGraph() if self.dirigido else nx.Graph()
        G.add_nodes_from(self.vertices)
        for (origem, destino), peso in self.arestas.items():
            G.add_edge(origem, destino, weight=peso)
        return G

    def mostraGrafo(self):
        """
        Desenha o grafo atual
        Basicamente os nx.draw sao pra melhorar a visibilidade do grafo
        Ta separando em componentes fortemente conexas tambem caso necessario, mas da pra tirar tranquilo (linha 41 ate 46)
        """
        G = self.geraGrafo()

        if not self.dirigido and nx.is_connected(G):
            components = [G] 
        elif self.dirigido and nx.is_weakly_connected(G):
            components = [G]  
        else:
            components = [G.subgraph(c).copy() for c in nx.weakly_connected_components(G)]

        num_components = len(components)
        fig, axes = plt.subplots(1, num_components, figsize=(15 * num_components, 15))
        if num_components == 1:
            axes = [axes]

        for i, component in enumerate(components):
            pos = nx.kamada_kawai_layout(component) 

            nx.draw(
                component,
                pos,
                with_labels=True,
                node_color='lightblue',
                font_size=8,
                node_size=500,
                font_weight='bold',
                edge_color='gray',
                ax=axes[i]
            )

            nx.draw_networkx_labels(
                component,
                pos,
                font_size=8,
                font_color="black",
                ax=axes[i]
            )

            edge_labels = nx.get_edge_attributes(component, "weight")
            nx.draw_networkx_edge_labels(
                component,
                pos,
                edge_labels=edge_labels,
                font_size=8,
                font_color="red",
                ax=axes[i]
            )

            axes[i].set_title(f"Componente {i+1}")

        plt.tight_layout()
   #     iniciar_thread()
        plt.show()

    def buscaProfundidade(self, vertice_inicial: str) -> list:
        visitados = []
        G = self.geraGrafo()

        def dfs(v):
            if v not in visitados:
                visitados.append(v)
                for vizinho in G.neighbors(v):
                    dfs(vizinho)

        dfs(vertice_inicial)
        return visitados

def screenshot_graph_region(salvar_como="imaout\\screenshot.png"):
    """
    Tira uma captura de tela da região definida por duas imagens de referência.
    :param imagem1: Caminho da imagem que marca o canto superior esquerdo.
    :param imagem2: Caminho da imagem que marca o canto inferior direito.
    :param salvar_como: Caminho e nome do arquivo onde salvar a captura.
    """
    try:
        p.click(p.locateCenterOnScreen(r'ima\\BTNmaximize.png', confidence=0.9, minSearchTime=15))
        p.sleep(2)
        p.screenshot(salvar_como)
        p.click(p.locateCenterOnScreen(r'ima\\BTNcloseWindow.png', confidence=0.9, minSearchTime=15))
    
    except Exception as e:
        print(f"Erro ao tirar captura de tela: {e}")


def iniciar_thread():
    """
    Inicia a função de captura de tela em uma thread separada.
    """
    thread = threading.Thread(target=screenshot_graph_region)
   # thread.start()
    return thread

def gerarGrafoRedesComputadores():
    rede = Grafo(dirigido=True)
    
    maximo = random.randint(5, 25)
    servidores = [f"{i}" for i in range(1, maximo)]

    print('Definindo os servidores da rede (vertices)')
    for servidor in servidores:
        rede.adicionarVertice(servidor)

    print('Definindo os pares de servidores que podem se comunicar (arestas)')
    for _ in range(50): 
        origem = random.choice(servidores)
        destino = random.choice(servidores)
        peso = random.uniform(1.0, 10.0)
        if origem != destino:  
            try:
                rede.adicionarAresta(origem, destino, peso=round(peso, 0))
            except ValueError:
                pass  

    print('Mostrando o grafo')
    rede.mostraGrafo()
 
    
    print('Executando a busca em profundidade(DFS) a partir de um vertice inicial definido (1)')
    vertice_inicial = input(f"Vértice inicial do DFS: (digite um número entre 1 e {maximo})")
    vertice_inicial.strip()
    resultado_dfs = rede.buscaProfundidade(vertice_inicial)

    print(f"Ordem de visitação a partir de {vertice_inicial}: {resultado_dfs}")


if __name__ == "__main__":
    gerarGrafoRedesComputadores()
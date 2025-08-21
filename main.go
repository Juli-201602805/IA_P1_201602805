package main

import (
	"container/heap"
	"fmt"
	"image/color"
	"math/rand"
	"strconv"
	"strings"
	"time"

	"fyne.io/fyne/v2"
	"fyne.io/fyne/v2/app"
	"fyne.io/fyne/v2/container"
	"fyne.io/fyne/v2/theme"
	"fyne.io/fyne/v2/widget"
)

const (
	TamanoCuadricula     = 3
	ValorEspacioVacio    = 0
	VelocidadAnimacionMs = 350
)

var ConfiguracionObjetivo = []int{1, 2, 3, 4, 5, 6, 7, 8, ValorEspacioVacio}

type TemaPastel struct{}

func (t *TemaPastel) Color(n fyne.ThemeColorName, v fyne.ThemeVariant) color.Color {
	if n == theme.ColorNamePrimary {
		// Morado pastel
		return color.NRGBA{R: 206, G: 147, B: 216, A: 255}
	}
	return theme.DarkTheme().Color(n, v)
}
func (t *TemaPastel) Font(s fyne.TextStyle) fyne.Resource     { return theme.DarkTheme().Font(s) }
func (t *TemaPastel) Icon(n fyne.ThemeIconName) fyne.Resource { return theme.DarkTheme().Icon(n) }
func (t *TemaPastel) Size(n fyne.ThemeSizeName) float32       { return theme.DarkTheme().Size(n) }

type EstadoPuzzle struct {
	valoresAzulejos       []int
	indiceEspacioEnBlanco int
}

func copiarArregloEnteros(origen []int) []int {
	destino := make([]int, len(origen))
	copy(destino, origen)
	return destino
}

func crearEstadoDesdeAzulejos(valores []int) EstadoPuzzle {
	estado := EstadoPuzzle{valoresAzulejos: copiarArregloEnteros(valores)}
	for i, v := range valores {
		if v == ValorEspacioVacio {
			estado.indiceEspacioEnBlanco = i
			break
		}
	}
	return estado
}

func (e EstadoPuzzle) obtenerClaveUnica() string {
	var b strings.Builder
	for i, v := range e.valoresAzulejos {
		if i > 0 {
			b.WriteByte(',')
		}
		b.WriteString(strconv.Itoa(v))
	}
	return b.String()
}

func (e EstadoPuzzle) esEstadoObjetivo() bool {
	for i := range e.valoresAzulejos {
		if e.valoresAzulejos[i] != ConfiguracionObjetivo[i] {
			return false
		}
	}
	return true
}

func obtenerIndicesVecinos(indice int) []int {
	vec := []int{}
	fila, col := indice/TamanoCuadricula, indice%TamanoCuadricula
	if fila > 0 {
		vec = append(vec, indice-TamanoCuadricula)
	}
	if fila < TamanoCuadricula-1 {
		vec = append(vec, indice+TamanoCuadricula)
	}
	if col > 0 {
		vec = append(vec, indice-1)
	}
	if col < TamanoCuadricula-1 {
		vec = append(vec, indice+1)
	}
	return vec
}

func (e EstadoPuzzle) generarEstadosVecinos() []EstadoPuzzle {
	estados := []EstadoPuzzle{}
	for _, j := range obtenerIndicesVecinos(e.indiceEspacioEnBlanco) {
		next := copiarArregloEnteros(e.valoresAzulejos)
		next[e.indiceEspacioEnBlanco], next[j] = next[j], next[e.indiceEspacioEnBlanco]
		estados = append(estados, crearEstadoDesdeAzulejos(next))
	}
	return estados
}

// ------------------- Heurística Manhattan --------------------

func calcularDistanciaManhattan(valores []int) int {
	total := 0
	for idx, valor := range valores {
		if valor == ValorEspacioVacio {
			continue
		}
		meta := valor - 1
		r1, c1 := idx/TamanoCuadricula, idx%TamanoCuadricula
		r2, c2 := meta/TamanoCuadricula, meta%TamanoCuadricula
		if r1 < r2 {
			total += r2 - r1
		} else {
			total += r1 - r2
		}
		if c1 < c2 {
			total += c2 - c1
		} else {
			total += c1 - c2
		}
	}
	return total
}

// ------------------- Búsqueda A* ------------------------------

type NodoBusqueda struct {
	estadoDelPuzzle       EstadoPuzzle
	costoRealG            int
	valorHeuristicoH      int
	valorTotalF           int
	nodoPadre             *NodoBusqueda
	indiceEnColaPrioridad int
}

type ColaPrioridadNodos []*NodoBusqueda

func (c ColaPrioridadNodos) Len() int { return len(c) }
func (c ColaPrioridadNodos) Less(i, j int) bool {
	if c[i].valorTotalF == c[j].valorTotalF {
		if c[i].valorHeuristicoH == c[j].valorHeuristicoH {
			return c[i].costoRealG > c[j].costoRealG
		}
		return c[i].valorHeuristicoH < c[j].valorHeuristicoH
	}
	return c[i].valorTotalF < c[j].valorTotalF
}
func (c ColaPrioridadNodos) Swap(i, j int) {
	c[i], c[j] = c[j], c[i]
	c[i].indiceEnColaPrioridad = i
	c[j].indiceEnColaPrioridad = j
}
func (c *ColaPrioridadNodos) Push(x interface{}) {
	n := x.(*NodoBusqueda)
	n.indiceEnColaPrioridad = len(*c)
	*c = append(*c, n)
}
func (c *ColaPrioridadNodos) Pop() interface{} {
	old := *c
	n := len(old)
	x := old[n-1]
	x.indiceEnColaPrioridad = -1
	*c = old[0 : n-1]
	return x
}

func reconstruirRuta(n *NodoBusqueda) []EstadoPuzzle {
	ruta := []EstadoPuzzle{}
	for cur := n; cur != nil; cur = cur.nodoPadre {
		ruta = append(ruta, cur.estadoDelPuzzle)
	}
	for i, j := 0, len(ruta)-1; i < j; i, j = i+1, j-1 {
		ruta[i], ruta[j] = ruta[j], ruta[i]
	}
	return ruta
}

func buscarConAEstrella(inicio EstadoPuzzle) ([]EstadoPuzzle, bool) {
	abierta := &ColaPrioridadNodos{}
	heap.Init(abierta)

	nodoInicio := &NodoBusqueda{
		estadoDelPuzzle:  inicio,
		costoRealG:       0,
		valorHeuristicoH: calcularDistanciaManhattan(inicio.valoresAzulejos),
	}
	nodoInicio.valorTotalF = nodoInicio.costoRealG + nodoInicio.valorHeuristicoH
	heap.Push(abierta, nodoInicio)

	cerrado := map[string]bool{}
	mejorG := map[string]int{inicio.obtenerClaveUnica(): 0}

	for abierta.Len() > 0 {
		actual := heap.Pop(abierta).(*NodoBusqueda)
		if actual.estadoDelPuzzle.esEstadoObjetivo() {
			return reconstruirRuta(actual), true
		}
		cerrado[actual.estadoDelPuzzle.obtenerClaveUnica()] = true

		for _, vecino := range actual.estadoDelPuzzle.generarEstadosVecinos() {
			key := vecino.obtenerClaveUnica()
			if cerrado[key] {
				continue
			}
			tentativoG := actual.costoRealG + 1
			if bg, ok := mejorG[key]; ok && tentativoG >= bg {
				continue
			}
			mejorG[key] = tentativoG

			next := &NodoBusqueda{
				estadoDelPuzzle:  vecino,
				costoRealG:       tentativoG,
				valorHeuristicoH: calcularDistanciaManhattan(vecino.valoresAzulejos),
				nodoPadre:        actual,
			}
			next.valorTotalF = next.costoRealG + next.valorHeuristicoH
			heap.Push(abierta, next)
		}
	}
	return nil, false
}

// ------------------- Desordenar n pasos ----------------------

func mezclarDesdeObjetivoConPasosValidos(pasos int) EstadoPuzzle {
	if pasos < 1 {
		pasos = 20
	}
	rand.Seed(time.Now().UnixNano())
	estado := crearEstadoDesdeAzulejos(ConfiguracionObjetivo)
	prev := -1

	for i := 0; i < pasos; i++ {
		opciones := obtenerIndicesVecinos(estado.indiceEspacioEnBlanco)
		if prev >= 0 && len(opciones) > 1 {
			filtradas := []int{}
			for _, o := range opciones {
				if o != prev {
					filtradas = append(filtradas, o)
				}
			}
			opciones = filtradas
		}
		eleccion := opciones[rand.Intn(len(opciones))]
		estado.valoresAzulejos[estado.indiceEspacioEnBlanco], estado.valoresAzulejos[eleccion] =
			estado.valoresAzulejos[eleccion], estado.valoresAzulejos[estado.indiceEspacioEnBlanco]
		prev = estado.indiceEspacioEnBlanco
		estado.indiceEspacioEnBlanco = eleccion
	}
	return estado
}

// ------------------- Interfaz  ----------------------

type InterfazGraficaPuzzle struct {
	ventanaPrincipal       fyne.Window
	botonesDeAzulejos      []*widget.Button
	etiquetaDeEstado       *widget.Label
	campoPasosParaMezcla   *widget.Entry
	botonIniciar           *widget.Button
	botonMezclar           *widget.Button
	botonResolver          *widget.Button
	botonResolverPasoAPaso *widget.Button
	estadoActualDelPuzzle  EstadoPuzzle
	rutaDeSolucion         []EstadoPuzzle
	indicePasoMostrado     int
}

func (ui *InterfazGraficaPuzzle) mostrarEstadoEnCuadricula(estado EstadoPuzzle) {
	ui.estadoActualDelPuzzle = estado
	for i, v := range estado.valoresAzulejos {
		txt := ""
		if v != ValorEspacioVacio {
			txt = strconv.Itoa(v)
		}
		ui.botonesDeAzulejos[i].SetText(txt)
	}
}

func (ui *InterfazGraficaPuzzle) deshabilitarControles(disabled bool) {
	if disabled {
		ui.botonIniciar.Disable()
		ui.botonMezclar.Disable()
		ui.botonResolver.Disable()
		ui.botonResolverPasoAPaso.Disable()
	} else {
		ui.botonIniciar.Enable()
		ui.botonMezclar.Enable()
		ui.botonResolver.Enable()
		ui.botonResolverPasoAPaso.Enable()
	}
}

func (ui *InterfazGraficaPuzzle) resolverDeFormaAsincrona(reproducir bool) {
	if ui.estadoActualDelPuzzle.esEstadoObjetivo() {
		ui.etiquetaDeEstado.SetText("Ya estás en el estado objetivo.")
		return
	}
	ui.deshabilitarControles(true)
	ui.etiquetaDeEstado.SetText("Buscando solución (A* + Manhattan)…")

	estadoInicial := ui.estadoActualDelPuzzle

	go func() {
		inicio := time.Now()
		ruta, ok := buscarConAEstrella(estadoInicial)
		duracion := time.Since(inicio)

		fyne.Do(func() {
			ui.deshabilitarControles(false)
			if !ok {
				ui.etiquetaDeEstado.SetText("No se encontró solución.")
				return
			}
			ui.rutaDeSolucion = ruta
			ui.indicePasoMostrado = 0
			ui.etiquetaDeEstado.SetText(
				fmt.Sprintf("Solución en %d pasos (%.2f ms).",
					len(ruta)-1, float64(duracion.Microseconds())/1000.0),
			)
			if reproducir {
				ui.reproducirRutaComoAnimacion()
			}
		})
	}()
}

func (ui *InterfazGraficaPuzzle) reproducirRutaComoAnimacion() {
	if len(ui.rutaDeSolucion) == 0 {
		return
	}
	go func() {
		for i := 0; i < len(ui.rutaDeSolucion); i++ {
			paso := ui.rutaDeSolucion[i]
			time.Sleep(time.Duration(VelocidadAnimacionMs) * time.Millisecond)
			fyne.Do(func() {
				ui.mostrarEstadoEnCuadricula(paso)
				ui.indicePasoMostrado = i
			})
		}
	}()
}

func (ui *InterfazGraficaPuzzle) avanzarUnPasoDeLaSolucion() {
	if len(ui.rutaDeSolucion) == 0 {
		ui.resolverDeFormaAsincrona(false)
		return
	}
	if ui.indicePasoMostrado < len(ui.rutaDeSolucion) {
		ui.mostrarEstadoEnCuadricula(ui.rutaDeSolucion[ui.indicePasoMostrado])
		ui.indicePasoMostrado++
	}
}

func main() {
	aplicacion := app.New()
	aplicacion.Settings().SetTheme(&TemaPastel{}) // morado pastel para los azulejos

	ventana := aplicacion.NewWindow("8-Puzzle (A* + Manhattan)")
	ventana.Resize(fyne.NewSize(560, 640))

	ui := &InterfazGraficaPuzzle{
		ventanaPrincipal: ventana,
		etiquetaDeEstado: widget.NewLabel("Listo."),
	}

	// --- Cuadrícula 3x3 de azulejos (HighImportance = morado pastel) ---
	contenedorCuadricula := container.NewGridWithColumns(TamanoCuadricula)
	for i := 0; i < TamanoCuadricula*TamanoCuadricula; i++ {
		indiceBoton := i
		boton := widget.NewButton("", func() {
			for _, vecino := range obtenerIndicesVecinos(ui.estadoActualDelPuzzle.indiceEspacioEnBlanco) {
				if vecino == indiceBoton {
					next := copiarArregloEnteros(ui.estadoActualDelPuzzle.valoresAzulejos)
					next[ui.estadoActualDelPuzzle.indiceEspacioEnBlanco], next[vecino] =
						next[vecino], next[ui.estadoActualDelPuzzle.indiceEspacioEnBlanco]
					ui.mostrarEstadoEnCuadricula(crearEstadoDesdeAzulejos(next))
					ui.rutaDeSolucion = nil
					ui.indicePasoMostrado = 0
					break
				}
			}
		})
		boton.Importance = widget.HighImportance // morado pastel
		ui.botonesDeAzulejos = append(ui.botonesDeAzulejos, boton)
		contenedorCuadricula.Add(boton)
	}

	// --- Contenido central  ---
	contenidoCentral := container.NewVBox(
		contenedorCuadricula,
		widget.NewSeparator(),
		ui.etiquetaDeEstado,
	)

	ui.campoPasosParaMezcla = widget.NewEntry()
	ui.campoPasosParaMezcla.SetPlaceHolder("n pasos (mezclar)")
	ui.campoPasosParaMezcla.SetText("30")

	ui.botonIniciar = widget.NewButton("Iniciar", func() {
		ui.mostrarEstadoEnCuadricula(crearEstadoDesdeAzulejos(ConfiguracionObjetivo))
		ui.rutaDeSolucion = nil
		ui.indicePasoMostrado = 0
		ui.etiquetaDeEstado.SetText("Estado objetivo cargado.")
	})
	ui.botonIniciar.Importance = widget.MediumImportance // gris

	ui.botonMezclar = widget.NewButton("Desordenar", func() {
		pasos, _ := strconv.Atoi(ui.campoPasosParaMezcla.Text)
		if pasos < 1 {
			pasos = 20
		}
		ui.mostrarEstadoEnCuadricula(mezclarDesdeObjetivoConPasosValidos(pasos))
		ui.rutaDeSolucion = nil
		ui.indicePasoMostrado = 0
		ui.etiquetaDeEstado.SetText(fmt.Sprintf("Desordenado en %d pasos válidos.", pasos))
	})
	ui.botonMezclar.Importance = widget.MediumImportance // gris

	ui.botonResolver = widget.NewButton("Resolver", func() { ui.resolverDeFormaAsincrona(true) })
	ui.botonResolver.Importance = widget.MediumImportance // gris

	ui.botonResolverPasoAPaso = widget.NewButton("Resolver paso a paso", func() { ui.avanzarUnPasoDeLaSolucion() })
	ui.botonResolverPasoAPaso.Importance = widget.MediumImportance // gris

	fila1 := container.NewGridWithColumns(2, ui.botonIniciar, ui.botonMezclar)
	fila2 := container.NewGridWithColumns(2, ui.campoPasosParaMezcla, ui.botonResolver)
	footer := container.NewVBox(
		widget.NewSeparator(),
		fila1,
		fila2,
		ui.botonResolverPasoAPaso,
	)

	root := container.NewBorder(nil, footer, nil, nil, contenidoCentral)
	ventana.SetContent(root)

	// Estado inicial
	ui.mostrarEstadoEnCuadricula(crearEstadoDesdeAzulejos(ConfiguracionObjetivo))
	ventana.ShowAndRun()
}
